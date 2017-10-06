classdef Joint < handle
    % JOINT object definition
    
    % JOINT object properties
    properties (SetAccess = private)
        X % 3D coordinates (in)
        ID % global node ID number
        dofs % degree-of-freedom IDs
    end
    
    % JOINT object methods
    methods
        % JOINT object constructor
        function JOINT = Joint(X, ID, dofs)
            JOINT.X = X;
            JOINT.ID = ID;
            JOINT.dofs = dofs;
        end
        
        % Get free JOINT dofs
        function [free, freeDofs] = free(JOINT)
            free = (JOINT.dofs > 0);
            freeDofs = JOINT.dofs(free);
        end
        
        % Get fixed JOINT dofs
        function [fixed, fixedDofs] = fixed(JOINT)
            fixed = (JOINT.dofs < 0);
            fixedDofs = -JOINT.dofs(fixed);
        end
    end
    
end

