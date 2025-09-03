function [ scores ] = computeEfficiency2026( weight, deflections, sways )
% COMPUTE EFFICIENCY:
%  calculate the overall efficiency scores from the
%  specified weight and aggregate deflections
%  (Consistent with the NSSBC 2026 rules)

% specify cost multipliers
C_w =      75; % ($/[lb^p_w])
p_w =     1.8;
C_d = 4000000; % ($/in)

% calculate lateral sway deflection factor gamma_lat:
%  gamma_lat = 0.9  if  sway <= 3/8 [in]
%  gamma_lat = 1.0  if  sway >  3/8 [in]
gamma_lat = 1.0 - 0.1*(sways <= 0.375);

% calculate overall score
scores = C_w * (weight^p_w) + C_d * (gamma_lat.*deflections);

end