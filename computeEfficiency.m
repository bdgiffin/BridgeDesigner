function [ score ] = computeEfficiency( weight, deflection )
% COMPUTE EFFICIENCY:
%  calculate the overall efficiency score from the
%  specified weight and aggregate deflection

% specify cost multipliers
C_w0 =    5000; % ($/lb)
C_w1 =   25000; % ($/lb)
C_d  = 3000000; % ($/in)

% calculate overall score
if (weight < 120)
    score = C_d * deflection;
elseif (weight < 200)
    score = C_d * deflection + C_w0 * (weight - 120);
else
    score = C_d * deflection + C_w1 * (weight - 184);
end

end