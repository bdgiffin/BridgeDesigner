classdef Frame < handle
    % FRAME object definition
    
    % FRAME object properties
    properties (SetAccess = public)
        elements
        material
        section
    end
    
    % FRAME object methods
    methods
        % FRAME object constructor
        function FRAME = Frame(joints, elements, material, section)
            FRAME.material = material;
            FRAME.section = section;
            FRAME.elements = cell(size(elements,1),1);
            for i = 1:size(elements,1)
                FRAME.elements{i} = Element(joints(elements(i,:)), FRAME.material, FRAME.section);
            end
        end
        
    end
    
end

