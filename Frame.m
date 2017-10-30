classdef Frame < handle
    % FRAME object definition
    
    % FRAME object properties
    properties (SetAccess = public)
        joints
        Nelements
        elements
        material
        section
    end
    
    % FRAME object methods
    methods
        % FRAME object constructor
        function FRAME = Frame(joints, frame, elements, material, section)
            FRAME.joints = joints(frame);
            FRAME.material = material;
            FRAME.section = copy(section);
            FRAME.Nelements = size(elements,1);
            FRAME.elements = cell(FRAME.Nelements,1);
            for i = 1:FRAME.Nelements
                FRAME.elements{i} = Element(joints(elements(i,:)), FRAME.material, FRAME.section);
            end
        end
        
    end
    
end

