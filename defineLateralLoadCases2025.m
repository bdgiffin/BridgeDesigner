function [ loads, measurements, probabilities ] = defineLateralLoadCases2025()
% DEFINE_LATERAL_LOAD_CASES:
%  define all lateral load cases for the model
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
Ncases = 11; % S(N=2-12) (lateral)
loads = cell(Ncases,1);
measurements = cell(Ncases,1);

% reference: +1 = measure from east (+x) end;
%            -1 = measure from west (-x) end
reference = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]; % S(N=2-12)

% side:   +1 = apply to left  / North (+y) side;
%         -1 = apply to right / South (-y) side
side = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]; % S(N=2-12)

% location: distance (in inches) to the lateral load & measurement location
locations = [ 7*12+0; ...  % S(N=2)
              7*12+0; ...  % S(N=3)
              7*12+0; ...  % S(N=4)
              7*12+0; ...  % S(N=5)
              8*12+6; ...  % S(N=6)
              8*12+6; ...  % S(N=7)
              8*12+6; ...  % S(N=8)
              7*12+0; ...  % S(N=9)
              7*12+0; ...  % S(N=10)
              7*12+0; ...  % S(N=11)
              7*12+0];     % S(N=12)

% weight: signed magnitude of the total applied lateral load (in pounds)
weight = 50 * side; % (lbs)
        
% assign load pattern definitions
for i = 1:Ncases
    loads{i} = [reference(i), side(i), locations(i), weight(i)];
    measurements{i} = [reference(i), side(i), locations(i)];
end

end