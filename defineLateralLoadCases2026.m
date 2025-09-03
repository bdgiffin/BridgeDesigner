function [ loads, measurements, probabilities ] = defineLateralLoadCases2026()
% DEFINE_LATERAL_LOAD_CASES:
%  define all lateral load cases for the model
%  (Consistent with the NSSBC 2026 rules)

% specify the "probabilities" of occurance for each die roll outcome
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
Ntests = 2;  % T1-2 (backspan & cantilever)
loads = cell(Ncases,Ntests);
measurements = cell(Ncases,Ntests);

% reference: +1 = measure from east (+x) end;
%            -1 = measure from west (-x) end
%            T1 T2 (backspan & cantilever)
reference = [-1,+1];

% side:   +1 = apply to left  / North (+y) side;
%         -1 = apply to right / South (-y) side
side = [-1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1]; % S(N=2-12)

% location: distance (in inches) to the lateral load & measurement location
%               T1       T2   (backspan & cantilever)
locations = [10*12+0,  0*12+1; ...  % N=2
             10*12+0,  0*12+1; ...  % N=3
             10*12+0,  0*12+1; ...  % N=4
             10*12+0,  0*12+1; ...  % N=5
             10*12+0,  0*12+1; ...  % N=6
             10*12+0,  0*12+1; ...  % N=7
             10*12+0,  0*12+1; ...  % N=8
             10*12+0,  0*12+1; ...  % N=9
             10*12+0,  0*12+1; ...  % N=10
             10*12+0,  0*12+1; ...  % N=11
             10*12+0,  0*12+1];     % N=12

% weight: signed magnitude of the total applied lateral load (in pounds)
weight = 50 * side; % (lbs)
        
% assign load pattern definitions
for i = 1:Ncases
    for j = 1:Ntests
        loads{i,j} = [reference(j), side(i), locations(i,j), weight(i)];
        measurements{i,j} = [reference(j), side(i), locations(i,j)];
    end
end

end