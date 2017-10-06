classdef Section < handle
    % SECTION object definition
    
    % SECTION object properties
    properties (SetAccess = public)
        name
        type
        A % cross-sectional area
        I % 2nd moment of aera
        J % polar moment of area
        c % furthest distance to extreme fiber
    end
    
    % SECTION object methods
    methods
        % SECTION object constructor
        function SECTION = Section(name, type, d, t)
            SECTION.name = strtrim(name);
            if strcmp(strtrim(type), 'PIPE')
                % circular pipe section
                %    d = outer diameter
                %    t = wall thickness
                SECTION.type = 'PIPE';
                SECTION.A = pi * ((d/2)^2-(d/2-t)^2);
                SECTION.I = pi * ((d/2)^4-(d/2-t)^4) / 4;
                SECTION.J = 2 * SECTION.I;
                SECTION.c = d/2;
            elseif strcmp(strtrim(type), 'TUBE')
                % square tube section
                %    d = width
                %    t = wall thickness
                SECTION.type = 'TUBE';
                SECTION.A = (d^2-(d-2*t)^2);
                SECTION.I = (d^4-(d-2*t)^4) / 12;
                SECTION.J = 2 * SECTION.I;
                SECTION.c = d/2;
            end
        end
    end
    
end

