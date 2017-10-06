function [ loads, measurements ] = defineLateralLoadCases()
% DEFINE_LATERAL_LOAD_CASES:
%  define all lateral load cases for the model

% define the load "pattern" matrix
Ncases = 12; % S1-6, mirrored (lateral)
loads = cell(Ncases,1);
measurements = cell(Ncases,1);

% reference: +1 = measure from east (+x) end;
%            -1 = measure from west (-x) end
reference = [+1, +1, +1, +1, +1, +1, ... % S1-S6
             -1, -1, -1, -1, -1, -1];    % S7-S12

% side:   +1 = apply to left  (+y) side;
%         -1 = apply to right (-y) side
side = [-1, -1, -1, -1, -1, -1, ... % S1-S6
        +1, +1, +1, +1, +1, +1];    % S7-S12

% location: distance (in inches) to the lateral load & measurement location
locations = [ 8*12+2; ...  % S1
              8*12+6; ...  % S2
             9*12+10; ...  % S3
             10*12+6; ...  % S4
             11*12+1; ...  % S5
             11*12+6; ...  % S6
              8*12+2; ...  % S1 (mirrored)
              8*12+6; ...  % S2 (mirrored)
             9*12+10; ...  % S3 (mirrored)
             10*12+6; ...  % S4 (mirrored)
             11*12+1; ...  % S5 (mirrored)
             11*12+6;];    % S6 (mirrored)

% weight: magnitude of the total applied lateral load (in pounds)
weight = 50 * side; % (lbs)
        
% assign load pattern definitions
for i = 1:Ncases
    loads{i} = [reference(i), side(i), locations(i), weight(i)];
    measurements{i} = [reference(i), side(i), locations(i)];
end

end