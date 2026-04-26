%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function [tau, confidence] = histBitSync(I_P, blockSize, minBlocks)
%function [tau, confidence] = histBitSync(I_P, blockSize, minBlocks)
%Histogram-based data bit synchronization for GPS L1 C/A signals.
%Finds the 20ms data bit boundary by maximizing the energy of coherent
%20ms sums across all 20 possible boundary offsets.
%
%[tau, confidence] = histBitSync(I_P, blockSize, minBlocks)
%
%   Inputs:
%       I_P         - Prompt In-Phase correlator outputs (row vector, 1xN)
%       blockSize   - Number of 1ms samples per data bit (20 for GPS L1 C/A)
%       minBlocks   - Minimum number of complete blocks required at an
%                     offset for it to be considered
%   Outputs:
%       tau         - Bit boundary offset (0..blockSize-1). Sample indices
%                     tau+1, tau+1+blockSize, tau+1+2*blockSize, ... mark
%                     the first 1ms sample of each new data bit.
%       confidence  - Ratio of best energy to mean of all other valid
%                     energies. Values > 2.0 indicate reliable detection.
%                     If confidence <= 1.0, tau is unreliable.
%
%Algorithm:
%   For each offset tau = 0..blockSize-1:
%     - Extract complete blockSize-sample blocks starting at sample tau+1
%     - For each block, compute coherent sum S = sum(I_P over the block)
%     - Accumulate energy E(tau) = sum(S^2) over all complete blocks
%   Return the tau with maximum E(tau) and the confidence ratio.
%
%Reference:
%   Parkinson & Spilker, "Global Positioning System: Theory and
%   Applications", Vol. I.
%
%--------------------------------------------------------------------------
% Copyright (C) D.M.Akos
% Written by Xin Zhang, based on the histogram method
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

N = length(I_P);
energyPerBlockVals = zeros(1, blockSize);
bestTau = 0;
bestEpb = -1;

for t = 0:(blockSize - 1)
    firstSample = t + 1;
    numBlocks = floor((N - firstSample + 1) / blockSize);

    if numBlocks < minBlocks
        continue;
    end

    energy = 0;
    for k = 0:(numBlocks - 1)
        startIdx = firstSample + k * blockSize;
        blockSum = sum(I_P(startIdx:startIdx + blockSize - 1));
        energy = energy + blockSum^2;
    end

    epb = energy / numBlocks;  % Energy per block, unbiased comparison
    energyPerBlockVals(t + 1) = epb;

    if epb > bestEpb
        bestEpb = epb;
        bestTau = t;
    end
end

tau = bestTau;

% Confidence: ratio of best energy per block to mean of others
validEpb = energyPerBlockVals(energyPerBlockVals > 0);
if length(validEpb) > 1
    otherMean = (sum(validEpb) - bestEpb) / (length(validEpb) - 1);
    if otherMean > 0
        confidence = bestEpb / otherMean;
    else
        confidence = 1.0;
    end
else
    confidence = 1.0;
end
