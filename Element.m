classdef Element < handle
    % ELEMENT object definition
    
    % ELEMENT object properties
    properties (SetAccess = private)
        joints
        material
        section
        
        % element dimensional parameters
        L % length (in)
        R % rotation
        
        % element dof parameters
        dofMap
        free
        freeDofs
        fixed
        fixedDofs
        
        % capacity
        capacity
        
    end
    
    % ELEMENT object methods
    methods
        % ELEMENT object constructor
        function ELEMENT = Element(joints, material, section)
            ELEMENT.joints = joints;
            ELEMENT.material = material;
            ELEMENT.section = section;
            ELEMENT.capacity = 0.0;
            ELEMENT = initialize(ELEMENT);
        end
        
        % Initialize ELEMENT object
        function ELEMENT = initialize(ELEMENT)
            % compute the length of the element
            d = ELEMENT.joints{2}.X - ELEMENT.joints{1}.X;
            ELEMENT.L = norm(d,2);

            % compute the rotation matrix
            t1 = atan2(d(3),norm(d(1:2),2));
            t2 = atan2(d(2),d(1));
            R1 = [cos(t1),      0.0,  -sin(t1); ...
                      0.0,      1.0,       0.0; ...
                  sin(t1),      0.0,   cos(t1)];
            R2 = [cos(t2), -sin(t2),       0.0; ...
                  sin(t2),  cos(t2),       0.0; ...
                      0.0,      0.0,       1.0];
            ELEMENT.R = R2*R1;
            
            % compute fixed/free dof maps and indicies
            ELEMENT.dofMap = [(1:6) + 6*(ELEMENT.joints{1}.ID - 1), ...
                              (1:6) + 6*(ELEMENT.joints{2}.ID - 1)]';
            dofs = [ELEMENT.joints{1}.dofs; ELEMENT.joints{2}.dofs];
            ELEMENT.free = (dofs > 0);
            ELEMENT.freeDofs = dofs(ELEMENT.free);
            ELEMENT.fixed = (dofs < 0);
            ELEMENT.fixedDofs = -dofs(ELEMENT.fixed);
        end
        
        % Compute ELEMENT stiffness matrix
        function [ weight ] = computeWeight(ELEMENT)
            % compute the weight of the element (lbs)
            weight = ELEMENT.material.rho * ELEMENT.L * ELEMENT.section.A;
        end % computeWeight
        
        % Compute ELEMENT stiffness matrix
        function [k] = computeStiffness(ELEMENT)
            %  Assemble the stiffness matrix for the prismatic beam element
            R = ELEMENT.R;

            % compute the local stiffness quantities, and fill in the matrix
            Kxx = ELEMENT.section.A * ELEMENT.material.E / ELEMENT.L;
            Kyy = 12 * ELEMENT.material.E * ELEMENT.section.I / ELEMENT.L^3;
            Kzz = Kyy;
            Kuu = ELEMENT.material.G * ELEMENT.section.J / ELEMENT.L;
            Kvz = 6 * ELEMENT.material.E * ELEMENT.section.I / ELEMENT.L^2;
            Kwy = Kvz;
            Kv1 = 4 * ELEMENT.material.E * ELEMENT.section.I / ELEMENT.L;
            Kw1 = Kv1;
            Kv2 = 2 * ELEMENT.material.E * ELEMENT.section.I / ELEMENT.L;
            Kw2 = Kv2;
            k = [Kxx,  0.0,  0.0,  0.0,  0.0,  0.0, -Kxx,  0.0,  0.0,  0.0,  0.0,  0.0; ...
                 0.0, +Kyy,  0.0,  0.0,  0.0, +Kwy,  0.0, -Kyy,  0.0,  0.0,  0.0, +Kwy; ...
                 0.0,  0.0, +Kzz,  0.0, -Kvz,  0.0,  0.0,  0.0, -Kzz,  0.0, -Kvz,  0.0; ...
                 0.0,  0.0,  0.0, +Kuu,  0.0,  0.0,  0.0,  0.0,  0.0, -Kuu,  0.0,  0.0; ...
                 0.0,  0.0, -Kvz,  0.0, +Kv1,  0.0,  0.0,  0.0, +Kvz,  0.0, +Kv2,  0.0; ...
                 0.0, +Kwy,  0.0,  0.0,  0.0, +Kw1,  0.0, -Kwy,  0.0,  0.0,  0.0, +Kw2; ...
                -Kxx,  0.0,  0.0,  0.0,  0.0,  0.0, +Kxx,  0.0,  0.0,  0.0,  0.0,  0.0; ...
                 0.0, -Kyy,  0.0,  0.0,  0.0, -Kwy,  0.0, +Kyy,  0.0,  0.0,  0.0, -Kwy; ...
                 0.0,  0.0, -Kzz,  0.0, +Kvz,  0.0,  0.0,  0.0, +Kzz,  0.0, +Kvz,  0.0; ...
                 0.0,  0.0,  0.0, -Kuu,  0.0,  0.0,  0.0,  0.0,  0.0, +Kuu,  0.0,  0.0; ...
                 0.0,  0.0, -Kvz,  0.0, +Kv2,  0.0,  0.0,  0.0, +Kvz,  0.0, +Kv1,  0.0; ...
                 0.0, +Kwy,  0.0,  0.0,  0.0, +Kw2,  0.0, -Kwy,  0.0,  0.0,  0.0, +Kw1];

            % rotate the stiffness matrix into the global coordinate system
            for i = 1:4
                ids = (1:3) + 3*(i-1);
                k(ids,:) = R * k(ids,:);
                k(:,ids) = k(:,ids) * R';
            end
        end
        
        % Compute ELEMENT geometric stiffness
        function [ g ] = computeGeometricStiffness(ELEMENT, displacements)
            % Assemble the geometric stiffness matrix for the prismatic beam element
            R = ELEMENT.R;

            % compute the internal tension force in the element
            delta = R(:,1)' * (displacements(7:9) - displacements(1:3));
            T = ELEMENT.material.E * ELEMENT.section.A * delta / ELEMENT.L;

            % compute the local geometric stiffness quantities, and fill in the matrix
            Kyy = 6 * T / (5 * ELEMENT.L);
            Kzz = Kyy;
            Kvz = T / 10;
            Kwy = Kvz;
            Kv1 = 2 * T * ELEMENT.L / 15;
            Kw1 = Kv1;
            Kv2 = T * ELEMENT.L / 30;
            Kw2 = Kv2;
            g = [0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0; ...
                 0.0, +Kyy,  0.0,  0.0,  0.0, +Kwy,  0.0, -Kyy,  0.0,  0.0,  0.0, +Kwy; ...
                 0.0,  0.0, +Kzz,  0.0, -Kvz,  0.0,  0.0,  0.0, -Kzz,  0.0, -Kvz,  0.0; ...
                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0; ...
                 0.0,  0.0, -Kvz,  0.0, +Kv1,  0.0,  0.0,  0.0, +Kvz,  0.0, -Kv2,  0.0; ...
                 0.0, +Kwy,  0.0,  0.0,  0.0, +Kw1,  0.0, -Kwy,  0.0,  0.0,  0.0, -Kw2; ...
                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0; ...
                 0.0, -Kyy,  0.0,  0.0,  0.0, -Kwy,  0.0, +Kyy,  0.0,  0.0,  0.0, -Kwy; ...
                 0.0,  0.0, -Kzz,  0.0, +Kvz,  0.0,  0.0,  0.0, +Kzz,  0.0, +Kvz,  0.0; ...
                 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0; ...
                 0.0,  0.0, -Kvz,  0.0, -Kv2,  0.0,  0.0,  0.0, +Kvz,  0.0, +Kv1,  0.0; ...
                 0.0, +Kwy,  0.0,  0.0,  0.0, -Kw2,  0.0, -Kwy,  0.0,  0.0,  0.0, +Kw1];

            % rotate the geometric stiffness matrix into the global coordinate system
            for i = 1:4
                ids = (1:3) + 3*(i-1);
                g(ids,:) = R * g(ids,:);
                g(:,ids) = g(:,ids) * R';
            end
        end % computeGeometricStiffness
        
        % Compute ELEMENT capacity
        function [ capacity ] = computeCapacity(ELEMENT, displacements)
            %  Assemble member capacity for the prismatic beam element
            R = ELEMENT.R;
            
            % compute the local element end-displacements and rotations
            u = zeros(3,2);
            q = zeros(3,2);
            for i = 1:2
                u(:,i) = R' * displacements((1:3)+6*(i-1));
                q(:,i) = R' * displacements((4:6)+6*(i-1));
            end
            
            % compute (constant) axial strain, stress
            ex = (u(1,2) - u(1,1)) / ELEMENT.L;
            sx = ELEMENT.material.E * ex;
            
            % compute (constant) axial twist, torsional shear stress
            rx = (q(1,2) - q(1,1)) / ELEMENT.L;
            tx = ELEMENT.material.G * rx * ELEMENT.section.c;
            
            % compute shape function 2nd derivatives at quadrature points
            x = [0.0, 1.0]; % sample at the end-points
            dphi_u(1,:) = (12*x - 6) / ELEMENT.L^2;
            dphi_q(1,:) = (6*x - 4) / ELEMENT.L;
            dphi_u(2,:) = (6 - 12*x) / ELEMENT.L^2;
            dphi_q(2,:) = (6*x - 2) / ELEMENT.L;
            
            % compute curvature, moment about the y and z axes
            ky = dphi_u(1,:) * u(2,1) + dphi_u(2,:) * u(2,2) ...
               + dphi_q(1,:) * q(3,1) + dphi_q(2,:) * q(3,2);
            kz = dphi_u(1,:) * u(3,1) + dphi_u(2,:) * u(3,2) ...
               - dphi_q(1,:) * q(2,1) - dphi_q(2,:) * q(2,2);
            My = ELEMENT.material.E * ELEMENT.section.I * ky;
            Mz = ELEMENT.material.E * ELEMENT.section.I * kz;
            
            % compute max bending stress
            if strcmp(ELEMENT.section.type, 'PIPE')
                M = sqrt(My.^2 + Mz.^2);
            elseif strcmp(ELEMENT.section.type, 'TUBE')
                M = abs(My) + abs(Mz);
            end
            sb = M * ELEMENT.section.c / ELEMENT.section.I;
            
            % check for gross yield (in tension or compression), due to
            % combined axial and bending stress
            yield_capacity = (abs(sx) + max(sb)) / ELEMENT.material.Fy;
            
            % check for gross yield in shear due only to torsion
            shear_capacity = abs(tx) / ELEMENT.material.Fy;
            
            % check for elastic buckling
            Fcr = ELEMENT.material.E * ELEMENT.section.I * (pi / ELEMENT.L)^2 / ELEMENT.section.A;
            buckling_capacity = (sx < 0) * abs(sx) / Fcr;
            
            % compute and store the overall member capacity
            capacity = max([yield_capacity, shear_capacity, buckling_capacity]);
            ELEMENT.capacity = max(ELEMENT.capacity, capacity);
            
        end
        
        % Plot deformed ELEMENT
        function plotDeformed(ELEMENT, displacements)
            % create cylinder
            r = ELEMENT.section.c;
            if strcmp(ELEMENT.section.type,'PIPE')
                [x,y,z] = cylinder(r * [1,1]);
            else
                [x0,y0,z] = cylinder(r * [1,1],4);
                x = x0 - y0;
                y = y0 + x0;
            end
            x1 = [z(1,:); y(1,:); x(1,:)];
            x2 = [z(2,:); y(2,:); x(2,:)];
            
            % scale cylinder
            p1 = ELEMENT.joints{1}.X' + displacements(1:3);
            p2 = ELEMENT.joints{2}.X' + displacements(7:9);
            d = p2 - p1;
            x2(1,:) = norm(d,2) * x2(1,:);
            
            % rotate cylinder
            t1 = atan2(d(3),norm(d(1:2),2));
            t2 = atan2(d(2),d(1));
            R1 = [cos(t1),      0.0,  -sin(t1); ...
                      0.0,      1.0,       0.0; ...
                  sin(t1),      0.0,   cos(t1)];
            R2 = [cos(t2), -sin(t2),       0.0; ...
                  sin(t2),  cos(t2),       0.0; ...
                      0.0,      0.0,       1.0];
            R = R2*R1;
            x1 = R * x1;
            x2 = R * x2;
            
            % translate cylinder
            x1 = x1 + repmat(p1,1,size(x1,2));
            x2 = x2 + repmat(p1,1,size(x2,2));
            
            % plot cylinder
            surf([x1(1,:); x2(1,:)],[x1(2,:); x2(2,:)],[x1(3,:); x2(3,:)], ...
                 'EdgeColor','none','FaceColor','b','FaceAlpha',0.6)
        end
        
        % Plot ELEMENT capacity
        function plotCapacity(ELEMENT)
            % create cylinder
            r = ELEMENT.section.c;
            if strcmp(ELEMENT.section.type,'PIPE')
                [x,y,z] = cylinder(r * [1,1]);
            else
                [x0,y0,z] = cylinder(r * [1,1],4);
                x = x0 - y0;
                y = y0 + x0;
            end
            x1 = [z(1,:); y(1,:); x(1,:)];
            x2 = [z(2,:); y(2,:); x(2,:)];
            
            % scale cylinder
            x2(1,:) = ELEMENT.L * x2(1,:);
            
            % rotate cylinder
            x1 = ELEMENT.R * x1;
            x2 = ELEMENT.R * x2;
            
            % translate cylinder
            x1 = x1 + repmat(ELEMENT.joints{1}.X',1,size(x1,2));
            x2 = x2 + repmat(ELEMENT.joints{1}.X',1,size(x2,2));
            
            % plot cylinder
            s = min(ELEMENT.capacity, 1.0);
            if (s >= 0)&&(s < 0.25)
                rgb = [0.0, 4.0 * s, 1.0];
            elseif (s >= 0.25)&&(s < 0.5)
                rgb = [0, 1.0, 1.0 - 4.0 * (s - 0.25)];
            elseif (s >= 0.5)&&(s < 0.75)
                rgb = [4.0 * (s - 0.5), 1.0, 0.0];
            else
                rgb = [1.0, 1.0 - 4.0 * (s - 0.75), 0.0];
            end
            surf([x1(1,:); x2(1,:)],[x1(2,:); x2(2,:)],[x1(3,:); x2(3,:)], ...
                 'EdgeColor','none','FaceColor',rgb)
        end
        
    end
    
end

