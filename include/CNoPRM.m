%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function [CNo] = CNoPRM(I, Q, T, M)
%function [CNo] = CNoPRM(I, Q, T, M)
%Calculate CNo using the Power Ratio Method (PRM).
%Compares wideband power to narrowband power to estimate SNR.
%
%[CNo] = CNoPRM(I, Q, T, M)
%
%   Inputs:
%       I           - Prompt In Phase values of the signal from Tracking
%       Q           - Prompt Quadrature Phase values of the signal from Tracking
%       T           - Accumulation interval in Tracking (in sec)
%       M           - Narrowband smoothing factor (K must be integral multiple of M)
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

%% Compute wideband power ====================================================
% Sum of instantaneous power over all K samples
WBP = sum(I.^2 + Q.^2);

%% Compute narrowband power ================================================
% Group M consecutive I and Q, sum each group, then square and accumulate
numGroups = K / M;

% Reshape I and Q into groups of M samples, then sum over each group
I_groups = sum(reshape(I, M, numGroups), 1);
Q_groups = sum(reshape(Q, M, numGroups), 1);

% Narrowband power: sum of squared group sums
NBP = sum(I_groups.^2 + Q_groups.^2);

%% Estimate C/No from the power ratio ========================================
P_ratio = WBP / NBP;

% Clamp P_ratio to avoid numerical instability
% P_ratio -> 1 means infinite SNR; P_ratio -> M means zero SNR
P_ratio = max(1.001, min(M - 0.001, P_ratio));

% Solve SNR from theoretical expectation:
%   E[P_ratio] = (M + SNR) / (1 + SNR)
%   => SNR = (M - P_ratio) / (P_ratio - 1)
SNR_linear = (M - P_ratio) / (P_ratio - 1);

% Convert to C/No in dB-Hz
CNo = 10 * log10(SNR_linear / T);
