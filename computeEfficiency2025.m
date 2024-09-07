function [ scores ] = computeEfficiency2025( weight, deflections, sways )
% COMPUTE EFFICIENCY:
%  calculate the overall efficiency scores from the
%  specified weight and aggregate deflections
%  (Consistent with the NSSBC 2025 rules)

% specify cost multipliers
C_w =      15; % ($/[lb^p_w])
p_w =    2.11;
C_d = 4250000; % ($/in)

% calculate lateral sway deflection factor gamma_lat:
%  gamma_lat = 0.95  if  sway <= 1/2 [in]
%  gamma_lat = 1.0   if  sway >  1/2 [in]
gamma_lat = 1.0 - 0.05*(sways <= 0.5);

% calculate overall score
scores = C_w * (weight^p_w) + C_d * (gamma_lat.*deflections);

end