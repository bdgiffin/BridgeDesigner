function [ scores ] = computeEfficiency( weight, deflections )
% COMPUTE EFFICIENCY:
%  calculate the overall efficiency scores from the
%  specified weight and aggregate deflections
%  (Consistent with the NSSBC 2019 rules)

% specify cost multipliers
C_w0 =    5000; % ($/lb)
C_w1 =   25000; % ($/lb)
C_d  = 3250000; % ($/in)

% calculate overall score
if (weight < 120)
    scores = C_d * deflections;
elseif (weight < 200)
    scores = C_d * deflections + C_w0 * (weight - 120);
else
    scores = C_d * deflections + C_w1 * (weight - 184);
end

end