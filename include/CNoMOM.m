%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function [CNo] = CNoMOM(I, Q, T)
%function [CNo] = CNoMOM(I, Q, T)
%Calculate CNo using the Moments Method (MOM).
%Uses the 2nd and 4th moments of the signal envelope to separate
%signal power from noise power.
%
%[CNo] = CNoMOM(I, Q, T)
%
%   Inputs:
%       I           - Prompt In Phase values of the signal from Tracking
%       Q           - Prompt Quadrature Phase values of the signal from Tracking
%       T           - Accumulation interval in Tracking (in sec)
%   Outputs:
%       CNo         - Estimated C/No for the given values of I and Q (dB-Hz)
%
%   Reference:
%       Pauluzzi & Beaulieu, "A Comparison of SNR Estimation Techniques
%       for the AWGN Channel", IEEE Trans. Comm., 2000.
%
%--------------------------------------------------------------------------
% Copyright (C) D.M.Akos
% Written by Sirish Jetti (VSM), extended with MOM method
%--------------------------------------------------------------------------
%This program is free software; you can redistribute it and/or
%modify it under the terms of the GNU General Public License
%as published by the Free Software Foundation; either version 2
%of the License, or (at your option) any later version.
%
%This program is distributed in the hope that it will be useful,
%but WITHOUT ANY WARRANTY; without even the implied warranty of
%MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%GNU General Public License for more details.
%
%You should have received a copy of the GNU General Public License
%along with this program; if not, write to the Free Software
%Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
%USA.
%--------------------------------------------------------------------------

%% Compute moments of the signal envelope ====================================
% Instantaneous power Z = I^2 + Q^2
Z = I.^2 + Q.^2;

% Second moment M2 = E[Z] = Pav + 2*sigma^2
M2 = mean(Z);

% Fourth moment M4 = E[Z^2]
M4 = mean(Z.^2);

%% Estimate C/No from the moment ratio ====================================
% For a Rician signal (signal + AWGN):
%   M2 = Pav + 2*sigma^2
%   M4 = Pav^2 + 4*Pav*sigma^2 + 8*sigma^4
%
% Define D = sqrt(2*M2^2 - M4). Then:
%   Pav = D
%   sigma^2 = (M2 - D) / 2
%   SNR = Pav / (2*sigma^2) = D / (M2 - D)

discriminant = 2 * M2^2 - M4;

if discriminant > 0
    D = sqrt(discriminant);
    % SNR_linear = Pav / (2*sigma^2) = D / (M2 - D)
    if M2 > D
        SNR_linear = D / (M2 - D);
    else
        SNR_linear = 0.01;  % Weak signal floor (~10 dB-Hz)
    end
else
    % Discriminant <= 0 due to noisy estimates: signal too weak to measure
    SNR_linear = 0.01;  % Reasonable floor instead of -30 dB-Hz artifact
end

% Convert to C/No in dB-Hz
CNo = 10 * log10(SNR_linear / T);
