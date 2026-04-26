%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function dataBits = decodeBitSigns(I_win, winGlobalStart, tau, blockSize)
%function dataBits = decodeBitSigns(I_win, winGlobalStart, tau, blockSize)
%Decode navigation data bit signs from a window of I_P correlator outputs,
%using a known bit boundary offset. Returns a ±1 sign vector at 1ms
%resolution suitable as the dataBits argument to CNoPRM.
%
%dataBits = decodeBitSigns(I_win, winGlobalStart, tau, blockSize)
%
%   Inputs:
%       I_win           - Window of I_P correlator outputs (row vector)
%       winGlobalStart  - Global sample index of I_win(1) relative to the
%                         start of tracking (1-based)
%       tau             - Bit boundary offset from histBitSync (0..blockSize-1)
%       blockSize       - Number of 1ms samples per data bit (20 for GPS L1)
%   Outputs:
%       dataBits        - Row vector of same length as I_win, with values
%                         +1 or -1. Complete 20-sample blocks aligned to
%                         the bit boundary receive the sign of their
%                         coherent sum. Partial blocks at window edges
%                         default to +1 (neutral).
%--------------------------------------------------------------------------
% Copyright (C) D.M.Akos
% Written by Xin Zhang
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

winLen = length(I_win);
dataBits = ones(1, winLen);

% Bit boundaries are at global samples: tau+1, tau+1+blockSize, ...
% Find the first complete block that starts at or after winGlobalStart
firstBitStart = tau + 1;
kFirst = ceil((winGlobalStart - firstBitStart) / blockSize);
if kFirst < 0
    kFirst = 0;
end

localStart = firstBitStart + kFirst * blockSize - winGlobalStart + 1;

while localStart <= winLen - blockSize + 1
    localEnd = localStart + blockSize - 1;
    blockSum = sum(I_win(localStart:localEnd));
    bitSign = 1;
    if blockSum < 0
        bitSign = -1;
    end
    dataBits(localStart:localEnd) = bitSign;
    localStart = localStart + blockSize;
end
