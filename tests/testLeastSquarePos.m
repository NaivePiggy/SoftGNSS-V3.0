function tests = testLeastSquarePos
%TESTLEASTSQUAREPOS Unit tests for receiver position least-squares solver.

tests = functiontests(localfunctions);
end

function setupOnce(~)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'geoFunctions'));
end

function testSyntheticPseudorangeSolution(testCase)
settings.c = 1e30;   % Suppress earth-rotation correction for exact geometry.
settings.IF = 38400;
settings.useTropCorr = 0;
settings.positioning.enableWeightedLS = 0;
settings.positioning.enableRobustLS = 0;

rxPos = [-2700000; -4300000; 3850000];
clockBias = 75000;

satXYZ = [ ...
    15600000, 18760000, 17610000, 19170000, 22100000, 23200000; ...
     7540000, 14630000,-13480000,   610000, -9600000, 12000000; ...
    20140000, 13480000, 13400000, 21300000, 14100000, -9000000];
satVel = zeros(3, size(satXYZ, 2));
satpos = [satXYZ; satVel];

obs = sqrt(sum((satXYZ - rxPos).^2, 1)) + clockBias;
freqforcal = settings.IF * ones(1, size(satXYZ, 2));

[posvel, ~, ~, dop] = leastSquarePos(satpos, obs, freqforcal, settings);

verifyLessThan(testCase, norm(posvel(1:3) - rxPos'), 1e-3);
verifyEqual(testCase, posvel(4), clockBias, 'AbsTol', 1e-3);
verifyEqual(testCase, posvel(5:8), zeros(1, 4), 'AbsTol', 1e-9);
verifyTrue(testCase, all(isfinite(dop)));
end
