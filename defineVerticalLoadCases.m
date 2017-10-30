function [ loads, measurements ] = defineVerticalLoadCases()
% DEFINE_VERTICAL_LOAD_CASES:
%  define all vertical load cases for the model

% define the load "pattern" matrix
Ncases = 12; % S1-6, (mirrored) 
Nloads = 2;  % L1 & L2
Nsteps = 3;  % Pre-load, L1, L2
loads = cell(Ncases, Nloads, Nsteps);

% reference: +1 = measure from east (+x) end;
%            -1 = measure from west (-x) end
reference = [+1, +1, +1, +1, +1, +1, ... % S1-S6
             -1, -1, -1, -1, -1, -1];    % S7-S12

% side(s):   +1 = apply to left  (+y) side;
%            -1 = apply to right (-y) side;
%             0 = apply to both sides
sides = 0; % apply to both sides

% location: distance (in inches) to the start of the distributed load
%               L1      L2
locations = [ 6*12+8, 1*12+0; ...  % S1
              7*12+0, 3*12+6; ...  % S2
              8*12+4, 3*12+0; ...  % S3
              9*12+0, 5*12+5; ...  % S4
              9*12+7, 3*12+4; ...  % S5
             10*12+0, 4*12+7; ...  % S6
              6*12+8, 1*12+0; ...  % S1 (mirrored)
              7*12+0, 3*12+6; ...  % S2 (mirrored)
              8*12+4, 3*12+0; ...  % S3 (mirrored)
              9*12+0, 5*12+5; ...  % S4 (mirrored)
              9*12+7, 3*12+4; ...  % S5 (mirrored)
             10*12+0, 4*12+7];     % S6 (mirrored)
         
% width: width (in inches) of the applied distributed loads
width = 36; % in

% weight: magnitude of the total applied distributed load (in pounds)
%          Pre-load   L1      L2
weights = [  150,    1550,   1550; ... % L1
              50,      50,   1050 ];   % L2
        
% assign load pattern definitions
for i = 1:Ncases
    for j = 1:Nloads
        for k = 1:Nsteps
            loads{i,j,k} = [reference(i), sides, locations(i,j), width, weights(j,k)];
        end
    end
end

% ----------------------------------------------------------------------- %

% define the deflection measurement matrix
Nmeasurements = 2; % D1 % D2
measurements = cell(Ncases, Nmeasurements);

% reference: +1 = measure from east (+x) end;
%            -1 = measure from west (-x) end
reference = [+1, +1, +1, +1, +1, +1, ... % S1-S6
             -1, -1, -1, -1, -1, -1];    % S7-S12

% side: +1 = measure on the left  (+y) side;
%       -1 = measure on the right (-y) side
side = [+1, +1, +1, +1, +1, +1, ... % S1-S6
        -1, -1, -1, -1, -1, -1];    % S7-S12

% location: distance (in inches) to the measurement location
% (shift the locations from earlier by 1'6")
%               D1      D2
locations = [ 8*12+2, 2*12+6;  ...  % S1
              8*12+6, 5*12+0;  ...  % S2
             9*12+10, 4*12+6;  ...  % S3
             10*12+6, 6*12+11; ...  % S4
             11*12+1, 4*12+10; ...  % S5
             11*12+6, 6*12+1;  ...  % S6
              8*12+2, 2*12+6;  ...  % S1 (mirrored)
              8*12+6, 5*12+0;  ...  % S2 (mirrored)
             9*12+10, 4*12+6;  ...  % S3 (mirrored)
             10*12+6, 6*12+11; ...  % S4 (mirrored)
             11*12+1, 4*12+10; ...  % S5 (mirrored)
             11*12+6, 6*12+1];      % S6 (mirrored)

% start/end_step: denotes the index of the load steps at which deflections
%                 will be zeroed (start) and measured (end)
start_step = 1;
end_step = 3;

% assign load pattern definitions
for i = 1:Ncases
    for j = 1:Nmeasurements
        measurements{i,j} = [reference(i), side(i), locations(i,j), start_step, end_step];
    end
end

end