%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function [posvel, el, az, dop] = leastSquarePos(satpos, obs, freqforcal, settings)
%Function calculates a weighted robust Least Square Solution.
%
%[posvel, el, az, dop] = leastSquarePos(satpos, obs, freqforcal, settings);
%
%   Inputs:
%       satpos      - Satellites positions and velocities in ECEF system:
%                   [X; Y; Z; Vx; Vy; Vz], one column per satellite
%       obs         - Pseudorange observations to each satellite
%       freqforcal  - Tracked carrier frequencies for velocity calculation
%       settings    - Receiver settings
%
%   Outputs:
%       posvel      - Receiver position, clock bias, velocity and clock
%                   drift in ECEF system: [X, Y, Z, dt, Vx, Vy, Vz, ddt]
%       el          - Satellites elevation angles (degrees)
%       az          - Satellites azimuth angles (degrees)
%       dop         - Dilutions Of Precision ([GDOP PDOP HDOP VDOP TDOP])

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
%--------------------------------------------------------------------------
%Based on Kai Borre
%Copyright (c) by Kai Borre
%Updated by Darius Plausinaitis, Peter Rinder and Nicolaj Bertelsen
%
% CVS record:
% $Id: leastSquarePos.m,v 1.1.2.12 2006/08/22 13:45:59 dpl Exp $
%==========================================================================

%% Initialization =========================================================
nmbOfIterations = 7;
dtr = pi / 180;

X = satpos(1:3, :);
vX = satpos(4:6, :);
nmbOfSatellites = size(satpos, 2);

pos = zeros(4, 1);
posvel = zeros(1, 8);
dop = zeros(1, 5);
az = zeros(1, nmbOfSatellites);
el = az;

if nmbOfSatellites < 4
    posvel = nan(1, 8);
    return
end

enableWeightedLS = getPositioningSetting(settings, ...
    'enableWeightedLS', 1) ~= 0;
enableRobustLS = getPositioningSetting(settings, ...
    'enableRobustLS', 1) ~= 0;
minElevationWeight = getPositioningSetting(settings, ...
    'minElevationWeight', 5);
minWeight = getPositioningSetting(settings, 'minWeight', 0.05);
robustTune = getPositioningSetting(settings, 'robustTune', 1.5);
robustMinSigma = getPositioningSetting(settings, ...
    'robustMinSigma', 10);

A = zeros(nmbOfSatellites, 4);
omc = zeros(nmbOfSatellites, 1);
weights = ones(nmbOfSatellites, 1);

%% Iteratively find receiver position ====================================
for iter = 1:nmbOfIterations

    for i = 1:nmbOfSatellites
        if iter == 1
            Rot_X = X(:, i);
            trop = 0;
        else
            rho2 = (X(1, i) - pos(1))^2 + ...
                   (X(2, i) - pos(2))^2 + ...
                   (X(3, i) - pos(3))^2;
            traveltime = sqrt(rho2) / settings.c;

            % Correct satellite position due to earth rotation.
            Rot_X = e_r_corr(traveltime, X(:, i));

            [az(i), el(i)] = topocent(pos(1:3, :), ...
                Rot_X - pos(1:3, :));

            if (settings.useTropCorr == 1)
                trop = tropo(sin(el(i) * dtr), ...
                    0.0, 1013.0, 293.0, 50.0, 0.0, 0.0, 0.0);
            else
                trop = 0;
            end
        end

        geometricRange = norm(Rot_X - pos(1:3), 'fro');
        omc(i) = obs(i) - geometricRange - pos(4) - trop;

        A(i, :) = [-(Rot_X(1) - pos(1)) / geometricRange, ...
                   -(Rot_X(2) - pos(2)) / geometricRange, ...
                   -(Rot_X(3) - pos(3)) / geometricRange, ...
                    1];
    end

    weights = ones(nmbOfSatellites, 1);
    if enableWeightedLS && iter > 1
        effectiveEl = max(el(:), minElevationWeight);
        weights = sin(effectiveEl * dtr) .^ 2;
        weights = max(weights, minWeight);
    end

    if enableRobustLS && iter > 2
        residualCenter = median(omc);
        sigma = 1.4826 * median(abs(omc - residualCenter));
        if ~isfinite(sigma) || sigma < robustMinSigma
            sigma = robustMinSigma;
        end

        huberLimit = robustTune * sigma;
        robustWeights = min(1, huberLimit ./ max(abs(omc), eps));
        weights = weights .* robustWeights;
    end

    sqrtWeights = sqrt(weights);
    weightedA = A .* repmat(sqrtWeights, 1, 4);
    weightedOmc = omc .* sqrtWeights;

    if rank(weightedA) ~= 4
        posvel = nan(1, 8);
        return
    end

    x = weightedA \ weightedOmc;
    pos = pos + x;

    if norm(x(1:3)) < 1e-4 && abs(x(4)) < 1e-4
        break
    end
end

pos = pos';

%% Calculate velocity from carrier frequency ==============================
a = zeros(nmbOfSatellites, 3);
d = zeros(nmbOfSatellites, 1);
HH = zeros(nmbOfSatellites, 4);

for i = 1:nmbOfSatellites
    r = norm(pos(1:3) - X(:, i)');
    if r == 0
        continue
    end

    satvel = vX(:, i)';
    a(i, :) = (X(:, i)' - pos(1:3)) / r;

    % Doppler convention (configurable; GN3S uses convention 2).
    if getPositioningSetting(settings, 'dopplerConvention', 2) == 1
        % Reversed:  f_doppler = -(freqforcal - IF)
        dopplerRangeRate = settings.c * (-freqforcal(i) + settings.IF) / ...
            1575.42e6;
    else
        % Normal:    f_doppler = freqforcal - IF
        dopplerRangeRate = settings.c * (freqforcal(i) - settings.IF) / ...
            1575.42e6;
    end
    d(i, 1) = dopplerRangeRate + sum(satvel .* a(i, :));

    HH(i, :) = [a(i, :), 1];
end

weightedHH = HH .* repmat(sqrtWeights, 1, 4);
weightedD = d .* sqrtWeights;
if rank(weightedHH) == 4
    vel = weightedHH \ weightedD;
else
    vel = nan(4, 1);
end

posvel = [pos, vel'];

%% Calculate Dilution Of Precision =======================================
if nargout == 4
    weightedAForDop = A .* repmat(weights, 1, 4);
    Q = pinv(A' * weightedAForDop);

    dop(1) = sqrt(trace(Q));                       % GDOP
    dop(2) = sqrt(Q(1, 1) + Q(2, 2) + Q(3, 3));   % PDOP
    dop(3) = sqrt(Q(1, 1) + Q(2, 2));             % HDOP
    dop(4) = sqrt(Q(3, 3));                       % VDOP
    dop(5) = sqrt(Q(4, 4));                       % TDOP
end

end

function value = getPositioningSetting(settings, fieldName, defaultValue)
value = defaultValue;

if isfield(settings, 'positioning') && ...
        isfield(settings.positioning, fieldName)
    value = settings.positioning.(fieldName);
elseif isfield(settings, fieldName)
    value = settings.(fieldName);
end

end
