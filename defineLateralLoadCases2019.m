function [ loads, measurements, probabilities ] = defineLateralLoadCases2019()
% DEFINE_LATERAL_LOAD_CASES:
%  define all lateral load cases for the model
%  (Consistent with the NSSBC 2019 rules)

% specify the "probabilities" of occurance for each load case
probabilities = [ 1/6, 1/6, 1/6, 1/6, 1/6, 1/6 ]; % S1-6

% define the load "pattern" matrix
Ncases = 6; % S1-6 (lateral)
loads = cell(Ncases,1);
measurements = cell(Ncases,1);

% reference: +1 = measure from east (+x) end;
%            -1 = measure from west (-x) end
reference = [+1, +1, +1, +1, +1, +1]; % S1-S6

% side:   +1 = apply to left  (+y) side;
%         -1 = apply to right (-y) side
side = [-1, -1, -1, -1, -1, -1]; % S1-S6

% location: distance (in inches) to the lateral load & measurement location
locations = [13*12+0; ...  % S1
             13*12+0; ...  % S2
              1*12+6; ...  % S3
              1*12+6; ...  % S4
              1*12+6; ...  % S5
             13*12+0];     % S6

% weight: signed magnitude of the total applied lateral load (in pounds)
weight = 50 * side; % (lbs)
        
% assign load pattern definitions
for i = 1:Ncases
    loads{i} = [reference(i), side(i), locations(i), weight(i)];
    measurements{i} = [reference(i), side(i), locations(i)];
end

end