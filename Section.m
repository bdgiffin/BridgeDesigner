classdef Section < handle
    % SECTION object definition
    
    % SECTION object properties
    properties (SetAccess = public)
        name
        type
        d
        t
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
            SECTION.d = d;
            SECTION.t = t;
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
        
        % SECTION distinct copy constructor
        function SECTION = copy(section)
            SECTION = Section(section.name, section.type, section.d, section.t);
        end
        
        % SECTION copy data method
        function copyData(SECTION, section)
            SECTION.name = section.name;
            SECTION.type = section.type;
            SECTION.d = section.d;
            SECTION.t = section.t;
            SECTION.A = section.A;
            SECTION.I = section.I;
            SECTION.J = section.J;
            SECTION.c = section.c;
        end
        
        % SECTION get properties method
        function [I,J,A,c] = getProperties(SECTION)
            I = SECTION.I;
            J = SECTION.J;
            A = SECTION.A;
            c = SECTION.c;
        end
    end
    
end

