classdef Material < handle
    % MATERIAL object definition
    
    % MATERIAL object properties
    properties (SetAccess = public)
        rho % mass density    (lbs/in^3)
        E   % Young's modulus (kips/in^2)
        nu  % Poisson's ratio (in/in)
        G   % Shear modulus   (ksi)
        Fy  % yield stress    (kips/in^2)
    end
    
    % MATERIAL object methods
    methods
        % MATERIAL object constructor
        function MATERIAL = Material(rho, E, nu, Fy)
            MATERIAL.rho = rho;
            MATERIAL.E   = E;
            MATERIAL.nu  = nu;
            MATERIAL.G   = E / (2 * (1 + nu));
            MATERIAL.Fy  = Fy;
        end
    end
    
end

