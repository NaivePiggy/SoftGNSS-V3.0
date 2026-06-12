%-----------------------------------------------------------------------------------
% This code has been adapted by Xin Zhang for purposes of course
% "AV423 Satellite Navigation" taught at School of Aeronautics & Astronautics,
% Shanghai Jiao Tong University,
% from the SoftGNSS v3.0 code base developed for the
% text: "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach"
% by Borre, Akos, et.al.
%-----------------------------------------------------------------------------------
function fllError = fllDiscriminator(prevI, prevQ, currI, currQ)
% Folded atan2 FLL discriminator for adjacent prompt correlator outputs.
%
%fllError = fllDiscriminator(prevI, prevQ, currI, currQ)
%
%   Inputs:
%       prevI, prevQ    - Previous prompt correlator I/Q values.
%       currI, currQ    - Current prompt correlator I/Q values.
%
%   Outputs:
%       fllError        - Phase change over one coherent integration interval
%                       in cycles. The output is amplitude-normalized and
%                       insensitive to 180-degree data bit polarity changes.

%--------------------------------------------------------------------------
%                           SoftGNSS v3.0
%
% Copyright (C) Dennis M. Akos
% Written by Darius Plausinaitis and Dennis M. Akos
% Based on code by DMAkos Oct-1999
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

dotProduct = prevI * currI + prevQ * currQ;
crossProduct = prevI * currQ - prevQ * currI;

if (dotProduct == 0) && (crossProduct == 0)
    fllError = 0;
else
    fllError = atan2(crossProduct, dotProduct) / (2.0 * pi);

    % Fold the phase difference into +/-0.25 cycles. This keeps the
    % discriminator insensitive to 180-degree navigation data bit flips.
    if fllError > 0.25
        fllError = fllError - 0.5;
    elseif fllError < -0.25
        fllError = fllError + 0.5;
    end
end
