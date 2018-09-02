function [ loads, measurements ] = defineVerticalLoadCases()
% DEFINE_VERTICAL_LOAD_CASES:
%  define all vertical load cases for the model
%  (Consistent with the NSSBC 2019 rules)

% define the load "pattern" matrix
Ncases = 6;  % S1-6
Nloads = 2;  % L1 & L2
Nsteps = 3;  % Pre-load, L1, L2
loads = cell(Ncases, Nloads, Nsteps);

% reference: +1 = measure from east (+x) end
%            -1 = measure from west (-x) end
reference = [+1, +1, +1, +1, +1, +1]; % S1-S6

% side(s):   +1 = apply to left  (+y) side
%            -1 = apply to right (-y) side
%             0 = apply to both sides
sides = 0; % apply to both sides

% location: distance (in inches) to the start of the distributed load
%               L1       L2
locations = [11*12+0,  6*12+0; ...  % S1
             12*12+0,  8*12+0; ...  % S2
             13*12+6, 10*12+0; ...  % S3
             14*12+0,  8*12+4; ...  % S4
             14*12+6, 10*12+8; ...  % S5
             15*12+5, 10*12+5];     % S6
         
% width: width (in inches) of the applied distributed loads
width = 36; % in

% weight: magnitude of the total applied distributed load (in pounds)
%          Pre-load   L1      L2
weights = [  150,    1550,   1550; ... % L1
             150,     150,   1050 ];   % L2
        
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

% reference: +1 = measure from east (+x) end
%            -1 = measure from west (-x) end
reference = [+1, +1, +1, +1, +1, +1]; % S1-S6

% side: +1 = measure on the left  (+y) side
%       -1 = measure on the right (-y) side
%       D1  D2
side = [+1, -1; ... % S1
        +1, -1; ... % S2
        +1, -1; ... % S3
        +1, -1; ... % S4
        +1, -1; ... % S5
        +1, -1];    % S6

% locations: distance (in inches) to the measurement locations
% (shift the locations from earlier by 1'6")
%               D1        D2
locations = [12*12+6,   7*12+6;  ...  % S1
             13*12+6,   9*12+6;  ...  % S2
             15*12+0,  11*12+6;  ...  % S3
             15*12+6,   9*12+10; ...  % S4
             16*12+0,  12*12+2;  ...  % S5
             16*12+11, 11*12+11];     % S6

% start/end_step: denotes the index of the load steps at which deflections
%                 will be zeroed (start) and measured (end)
start_step = 1;
end_step = 3;

% assign load pattern definitions
for i = 1:Ncases
    for j = 1:Nmeasurements
        measurements{i,j} = [reference(i), side(i,j), locations(i,j), start_step, end_step];
    end
end

end