function [ loads, measurements, probabilities ] = defineVerticalLoadCases2025()
% DEFINE_VERTICAL_LOAD_CASES:
%  define all vertical load cases for the model
%  (Consistent with the NSSBC 2025 rules)

% specify the "probabilities" of occurance for each load case
probabilities = [ 1/36, ...  % N=2
                  2/36, ...  % N=3
                  3/36, ...  % N=4
                  4/36, ...  % N=5
                  5/36, ...  % N=6
                  6/36, ...  % N=7
                  5/36, ...  % N=8
                  4/36, ...  % N=9
                  3/36, ...  % N=10
                  2/36, ...  % N=11
                  1/36];     % N=12

% define the load "pattern" matrix
Ncases = 11; % N=2-12
Nloads = 2;  % L1 & L2
Nsteps = 3;  % Pre-load, L1, L2
loads = cell(Ncases, Nloads, Nsteps);

% reference: +1 = measure from east (+x) end
%            -1 = measure from west (-x) end
reference = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]; % N=2-12

% side(s):   +1 = apply to left  / North (+y) side
%            -1 = apply to right / South (-y) side
%             0 = apply to both sides
sides = 0; % apply to both sides

% location: distance (in inches) to the start of the distributed load
%               L1       L2
locations = [ 3*12+0,  8*12+6; ...  % N=2
              3*12+6,  8*12+0; ...  % N=3
              4*12+0,  9*12+6; ...  % N=4
              4*12+6,  9*12+0; ...  % N=5
              5*12+0, 10*12+0; ...  % N=6
              6*12+0, 10*12+6; ...  % N=7
             10*12+0,  5*12+0; ...  % N=8
              9*12+0,  4*12+6; ...  % N=9
              9*12+6,  4*12+0; ...  % N=10
              8*12+0,  3*12+6; ...  % N=11
              8*12+6,  2*12+0];     % N=12
         
% width: width (in inches) of the applied distributed loads
width = 36; % in

% weight: magnitude of the total applied distributed load (in pounds)
%          Pre-load   L1      L2
weights = [  150,    1450,   1450; ... % L1
             150,     150,   1150 ];   % L2
        
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
reference = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]; % N=2-12

% side: +1 = measure on the left  / North (+y) side
%       -1 = measure on the right / South (-y) side
%       D1  D2
side = [+1, -1; ... % N=2
        +1, -1; ... % N=3
        +1, -1; ... % N=4
        +1, -1; ... % N=5
        +1, -1; ... % N=6
        +1, -1; ... % N=7
        +1, -1; ... % N=8
        +1, -1; ... % N=9
        +1, -1; ... % N=10
        +1, -1; ... % N=11
        +1, -1];    % N=12

% locations: distance (in inches) to the measurement locations
% (shift the locations from earlier by 1'6")
locations = locations + (1*12+6);

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
