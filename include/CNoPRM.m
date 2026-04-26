%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function [CNo] = CNoPRM(I, Q, T, M, dataBits)
%function [CNo] = CNoPRM(I, Q, T, M, dataBits)
%Calculate CNo using the Power Ratio Method (PRM).
%Compares wideband power to narrowband power to estimate SNR.
%
%[CNo] = CNoPRM(I, Q, T, M)
%[CNo] = CNoPRM(I, Q, T, M, dataBits)
%
%   Inputs:
%       I           - Prompt In Phase values of the signal from Tracking
%       Q           - Prompt Quadrature Phase values of the signal from Tracking
%       T           - Accumulation interval in Tracking (in sec)
%       M           - Narrowband smoothing factor (K must be integral multiple of M)
%       dataBits    - (Optional) Data bit signs, same length as I and Q.
%                     Values must be +1 or -1. When provided, both I and Q
%                     are multiplied by dataBits before narrowband coherent
%                     integration, stripping the navigation data modulation.
%   Outputs:
%       CNo         - Estimated C/No for the given values of I and Q (dB-Hz)
%
%   Reference:
%       Parkinson & Spilker, "Global Positioning System: Theory and
%       Applications", Vol. I, Chapter 8 (Signal Characteristics).
%
%--------------------------------------------------------------------------
% Copyright (C) D.M.Akos
% Written by Sirish Jetti (VSM), extended with PRM method
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

K = length(I);

%% Compute narrowband power first, then the ratio ===========================
% The theoretical expectation for the ratio NBP/WBP is:
%   E[NBP/WBP] = (M * SNR + 1) / (SNR + 1)
% where SNR = Pav / (2*sigma^2)

%% Strip data bits if provided (both I and Q) ============================
if nargin >= 5 && ~isempty(dataBits)
    I_nbp = I .* dataBits;
    Q_nbp = Q .* dataBits;
else
    I_nbp = I;
    Q_nbp = Q;
end

%% Compute wideband power ====================================================
WBP = sum(I.^2 + Q.^2);

%% Compute narrowband power ================================================
numGroups = K / M;

I_groups = sum(reshape(I_nbp, M, numGroups), 1);
Q_groups = sum(reshape(Q_nbp, M, numGroups), 1);
NBP = sum(I_groups.^2 + Q_groups.^2);

%% Estimate C/No from the power ratio ========================================
% NBP/WBP ranges from 1 (pure noise) to M (pure signal)
P_ratio = NBP / WBP;

% Clamp P_ratio to avoid numerical instability
P_ratio = max(1.001, min(M - 0.001, P_ratio));

% Solve SNR from: P_ratio = (M * SNR + 1) / (SNR + 1)
%   => SNR = (P_ratio - 1) / (M - P_ratio)
SNR_linear = (P_ratio - 1) / (M - P_ratio);

% Convert to C/No in dB-Hz
CNo = 10 * log10(SNR_linear / T);
