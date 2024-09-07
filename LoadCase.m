classdef LoadCase < handle
    % LOADCASE object definition
    
    % LOADCASE object properties
    properties (SetAccess = private)
        loads
        measurements
        intervals
        dofMap
    end % private properties

    properties (SetAccess = public)
        probability
    end % public properties
    
    % LOADCASE object methods
    methods
        
        % =============================================================== %
        
        % LOADCASE object constructor
        function LOADCASE = LoadCase(Nnodes, Nsteps, Nmeasurements, dofMap, probability)
            LOADCASE.loads = zeros(6*Nnodes, Nsteps);
            LOADCASE.measurements = zeros(6*Nnodes, Nmeasurements);
            LOADCASE.intervals = zeros(Nmeasurements, Nsteps);
            LOADCASE.dofMap = dofMap;
            LOADCASE.probability = probability;
        end % LoadCase
        
        % =============================================================== %
        
        % Assign vertical distributed loads to the LOADCASE
        function assignVerticalLoads(LOADCASE, patterns, nodes, elements)
            % find all members on the left (+y) and right (-y) sides of the bridge
            left  = elements(nodes(elements(:,1),2) > 0.0,:);
            right = elements(nodes(elements(:,1),2) < 0.0,:);

            % get the max and min span coordinates
            xmin = min(min(nodes(elements(:,1),1)), min(nodes(elements(:,2),1)));
            xmax = max(max(nodes(elements(:,1),1)), max(nodes(elements(:,2),1)));

            % define the equivalent joint "loads" using fixed-end moments
            Nloads = size(patterns,2);
            Nsteps = size(patterns,3);
            for k = 1:Nsteps
                for j = 1:Nloads
                    ref = patterns{1,j,k}(1);
                    sides = patterns{1,j,k}(2);
                    q = 0.001 * patterns{1,j,k}(5) / patterns{1,j,k}(4);
                    if (sides == 0)
                        % divide the load between both sides of the bridge evenly
                        q = q / 2;
                    end
                    if (ref == +1)
                        loc = xmax;
                    elseif (ref == -1)
                        loc = xmin;
                    end
                    if (sides >= 0)
                        for l = 1:size(left,1)
                            if (any(abs(nodes(left(l,:),1) - loc) > patterns{1,j,k}(3))&& ...
                                any(abs(nodes(left(l,:),1) - loc) < patterns{1,j,k}(3)+patterns{1,j,k}(4)))
                                e1 = min(abs(nodes(left(l,:),1) - loc));
                                e2 = max(abs(nodes(left(l,:),1) - loc));
                                p1 = max(e1,patterns{1,j,k}(3));
                                p2 = min(e2,patterns{1,j,k}(3)+patterns{1,j,k}(4));
                                L = e2 - e1;
                                d = p2 - p1;
                                a = p1 - e1 + d/2;
                                b = L - a;
                                M1 = (q*d/L^2)*(a*b^2+(a-2*b)*(d^2)/12);
                                M2 = (q*d/L^2)*(b*a^2+(b-2*a)*(d^2)/12);
                                R1 = (q*d/L^3)*((2*a+L)*b^2+0.25*(a-b)*d^2);
                                R2 = (q*d/L^3)*((2*b+L)*a^2-0.25*(a-b)*d^2);
                                if (ref == +1)
                                    [~,i1] = max(nodes(left(l,:),1));
                                    [~,i2] = min(nodes(left(l,:),1));
                                elseif (ref == -1)
                                    [~,i2] = max(nodes(left(l,:),1));
                                    [~,i1] = min(nodes(left(l,:),1));
                                end
                                ids1 = (1:6) + 6*(left(l,i1)-1);
                                ids2 = (1:6) + 6*(left(l,i2)-1);
                                LOADCASE.loads(ids1,k) = LOADCASE.loads(ids1,k) - [0, 0, R1, 0, +ref*M1, 0]';
                                LOADCASE.loads(ids2,k) = LOADCASE.loads(ids2,k) - [0, 0, R2, 0, -ref*M2, 0]';
                            end
                        end
                    end
                    if (sides <= 0)
                        for l = 1:size(right,1)
                            if (any(abs(nodes(right(l,:),1) - loc) > patterns{1,j,k}(3))&& ...
                                any(abs(nodes(right(l,:),1) - loc) < patterns{1,j,k}(3)+patterns{1,j,k}(4)))
                                e1 = min(abs(nodes(right(l,:),1) - loc));
                                e2 = max(abs(nodes(right(l,:),1) - loc));
                                p1 = max(e1,patterns{1,j,k}(3));
                                p2 = min(e2,patterns{1,j,k}(3)+patterns{1,j,k}(4));
                                L = e2 - e1;
                                d = p2 - p1;
                                a = p1 - e1 + d/2;
                                b = L - a;
                                M1 = (q*d/L^2)*(a*b^2+(a-2*b)*(d^2)/12);
                                M2 = (q*d/L^2)*(b*a^2+(b-2*a)*(d^2)/12);
                                R1 = (q*d/L^3)*((2*a+L)*b^2+0.25*(a-b)*d^2);
                                R2 = (q*d/L^3)*((2*b+L)*a^2-0.25*(a-b)*d^2);
                                if (ref == +1)
                                    [~,i1] = max(nodes(right(l,:),1));
                                    [~,i2] = min(nodes(right(l,:),1));
                                elseif (ref == -1)
                                    [~,i2] = max(nodes(right(l,:),1));
                                    [~,i1] = min(nodes(right(l,:),1));
                                end
                                ids1 = (1:6) + 6*(right(l,i1)-1);
                                ids2 = (1:6) + 6*(right(l,i2)-1);
                                LOADCASE.loads(ids1,k) = LOADCASE.loads(ids1,k) - [0, 0, R1, 0, +ref*M1, 0]';
                                LOADCASE.loads(ids2,k) = LOADCASE.loads(ids2,k) - [0, 0, R2, 0, -ref*M2, 0]';
                            end
                        end
                    end
                end
            end
            
        end % assignVerticalLoads
        
        % =============================================================== %
        
        % Assign vertical measurements to the LOADCASE
        function assignVerticalMeasurements(LOADCASE, points, nodes, elements)
            %  define equivalent joint measurements to indirectly
            %  measure vertical deflections at the specified points

            % find all members on the left (+y) and right (-y) sides of the bridge
            left  = elements(nodes(elements(:,1),2) > 0.0,:);
            right = elements(nodes(elements(:,1),2) < 0.0,:);

            % get the max and min span coordinates
            xmin = min(min(nodes(elements(:,1),1)), min(nodes(elements(:,2),1)));
            xmax = max(max(nodes(elements(:,1),1)), max(nodes(elements(:,2),1)));

            % define the equivalent joint "measurements" matrix using Hermite interpolation
            Nmeasurements = size(points,2);
            for j = 1:Nmeasurements
                ref = points{1,j}(1);
                if (ref == +1)
                    loc = xmax;
                elseif (ref == -1)
                    loc = xmin;
                end
                if (points{1,j}(2) > 0)
                    for l = 1:length(left)
                        if (any(abs(nodes(left(l,:),1) - loc) >= points{1,j}(3))&& ...
                            any(abs(nodes(left(l,:),1) - loc) <= points{1,j}(3)))
                            e1 = min(abs(nodes(left(l,:),1) - loc));
                            e2 = max(abs(nodes(left(l,:),1) - loc));
                            p = points{1,j}(3);
                            L = e2 - e1;
                            x = (p - e1)/L;
                            h00 = (2*x^3 - 3*x^2 + 1);
                            h10 = L * (x^3 - 2*x^2 + x);
                            h01 = (-2*x^3 + 3*x^2);
                            h11 = L * (x^3 - x^2);
                            if (ref == +1)
                                [~,i1] = max(nodes(left(l,:),1));
                                [~,i2] = min(nodes(left(l,:),1));
                            elseif (ref == -1)
                                [~,i2] = max(nodes(left(l,:),1));
                                [~,i1] = min(nodes(left(l,:),1));
                            end
                            ids1 = (1:6) + 6*(left(l,i1)-1);
                            ids2 = (1:6) + 6*(left(l,i2)-1);
                            LOADCASE.measurements(ids1,j) = LOADCASE.measurements(ids1,j) ...
                                                          + [0, 0, h00, 0, ref*h10, 0]';
                            LOADCASE.measurements(ids2,j) = LOADCASE.measurements(ids2,j) ...
                                                          + [0, 0, h01, 0, ref*h11, 0]';
                            LOADCASE.intervals(j,points{1,j}(4:5)) = [-1, +1];
                            break
                        end
                    end
                elseif (points{1,j}(2) < 0)
                    for l = 1:length(right)
                        if (any(abs(nodes(right(l,:),1) - loc) >= points{1,j}(3))&& ...
                            any(abs(nodes(right(l,:),1) - loc) <= points{1,j}(3)))
                            e1 = min(abs(nodes(right(l,:),1) - loc));
                            e2 = max(abs(nodes(right(l,:),1) - loc));
                            p = points{1,j}(3);
                            L = e2 - e1;
                            x = (p - e1)/L;
                            h00 = (2*x^3 - 3*x^2 + 1);
                            h10 = L * (x^3 - 2*x^2 + x);
                            h01 = (-2*x^3 + 3*x^2);
                            h11 = L * (x^3 - x^2);
                            if (ref == +1)
                                [~,i1] = max(nodes(right(l,:),1));
                                [~,i2] = min(nodes(right(l,:),1));
                            elseif (ref == -1)
                                [~,i2] = max(nodes(right(l,:),1));
                                [~,i1] = min(nodes(right(l,:),1));
                            end
                            ids1 = (1:6) + 6*(right(l,i1)-1);
                            ids2 = (1:6) + 6*(right(l,i2)-1);
                            LOADCASE.measurements(ids1,j) = LOADCASE.measurements(ids1,j) ...
                                                          + [0, 0, h00, 0, ref*h10, 0]';
                            LOADCASE.measurements(ids2,j) = LOADCASE.measurements(ids2,j) ...
                                                          + [0, 0, h01, 0, ref*h11, 0]';
                            LOADCASE.intervals(j,points{1,j}(4:5)) = [-1, +1];
                            break
                        end
                    end
                end
            end

        end

        % =============================================================== %
        
        % Assign lateral point load to the LOADCASE
        function assignLateralLoad(LOADCASE, pattern, nodes, elements)
            % find all members on the left (+y) and right (-y) sides of the bridge
            left  = elements(nodes(elements(:,1),2) > 0.0,:);
            right = elements(nodes(elements(:,1),2) < 0.0,:);

            % get the max and min span coordinates
            xmin = min(min(nodes(elements(:,1),1)), min(nodes(elements(:,2),1)));
            xmax = max(max(nodes(elements(:,1),1)), max(nodes(elements(:,2),1)));

            % define the equivalent joint "loads" using fixed-end moments
            ref = pattern(1);
            side = pattern(2);
            P = 0.001 * pattern(4);
            if (ref == +1)
                loc = xmax;
            elseif (ref == -1)
                loc = xmin;
            end
            if (side > 0)
                for l = 1:size(left,1)
                    if (any(abs(nodes(left(l,:),1) - loc) >= pattern(3))&& ...
                        any(abs(nodes(left(l,:),1) - loc) <= pattern(3)))
                        e1 = min(abs(nodes(left(l,:),1) - loc));
                        e2 = max(abs(nodes(left(l,:),1) - loc));
                        L = e2 - e1;
                        a = pattern(3) - e1;
                        b = L - a;
                        M1 = (P/L^2)*(a*b^2);
                        M2 = (P/L^2)*(b*a^2);
                        R1 = (P/L^3)*((3*a+b)*b^2);
                        R2 = (P/L^3)*((3*b+a)*a^2);
                        if (ref == +1)
                            [~,i1] = max(nodes(left(l,:),1));
                            [~,i2] = min(nodes(left(l,:),1));
                        elseif (ref == -1)
                            [~,i2] = max(nodes(left(l,:),1));
                            [~,i1] = min(nodes(left(l,:),1));
                        end
                        ids1 = (1:6) + 6*(left(l,i1)-1);
                        ids2 = (1:6) + 6*(left(l,i2)-1);
                        LOADCASE.loads(ids1,1) = LOADCASE.loads(ids1,1) + [0, R1, 0, 0, 0, -ref*M1]';
                        LOADCASE.loads(ids2,1) = LOADCASE.loads(ids2,1) + [0, R2, 0, 0, 0, +ref*M2]';
                        break
                    end
                end
            elseif (side < 0)
                for l = 1:size(right,1)
                    if (any(abs(nodes(right(l,:),1) - loc) >= pattern(3))&& ...
                        any(abs(nodes(right(l,:),1) - loc) <= pattern(3)))
                        e1 = min(abs(nodes(right(l,:),1) - loc));
                        e2 = max(abs(nodes(right(l,:),1) - loc));
                        L = e2 - e1;
                        a = pattern(3) - e1;
                        b = L - a;
                        M1 = (P/L^2)*(a*b^2);
                        M2 = (P/L^2)*(b*a^2);
                        R1 = (P/L^3)*((3*a+b)*b^2);
                        R2 = (P/L^3)*((3*b+a)*a^2);
                        if (ref == +1)
                            [~,i1] = max(nodes(right(l,:),1));
                            [~,i2] = min(nodes(right(l,:),1));
                        elseif (ref == -1)
                            [~,i2] = max(nodes(right(l,:),1));
                            [~,i1] = min(nodes(right(l,:),1));
                        end
                        ids1 = (1:6) + 6*(right(l,i1)-1);
                        ids2 = (1:6) + 6*(right(l,i2)-1);
                        LOADCASE.loads(ids1,1) = LOADCASE.loads(ids1,1) + [0, R1, 0, 0, 0, -ref*M1]';
                        LOADCASE.loads(ids2,1) = LOADCASE.loads(ids2,1) + [0, R2, 0, 0, 0, +ref*M2]';
                        break
                    end
                end
            end
            
        end % assignLateralLoad
        
        % =============================================================== %
        
        % Assign lateral measurement to the LOADCASE
        function assignLateralMeasurement(LOADCASE, point, nodes, elements)
            %  define equivalent joint measurements to indirectly
            %  measure lateral deflections at the specified points

            % find all members on the left (+y) and right (-y) sides of the bridge
            left  = elements(nodes(elements(:,1),2) > 0.0,:);
            right = elements(nodes(elements(:,1),2) < 0.0,:);

            % get the max and min span coordinates
            xmin = min(min(nodes(elements(:,1),1)), min(nodes(elements(:,2),1)));
            xmax = max(max(nodes(elements(:,1),1)), max(nodes(elements(:,2),1)));

            % define the equivalent joint "measurements" matrix using Hermite interpolation
            ref = point(1);
            side = point(2);
            if (ref == +1)
                loc = xmax;
            elseif (ref == -1)
                loc = xmin;
            end
            if (side > 0)
                for l = 1:length(left)
                    if (any(abs(nodes(left(l,:),1) - loc) >= point(3))&& ...
                        any(abs(nodes(left(l,:),1) - loc) <= point(3)))
                        e1 = min(abs(nodes(left(l,:),1) - loc));
                        e2 = max(abs(nodes(left(l,:),1) - loc));
                        p = point(3);
                        L = e2 - e1;
                        x = (p - e1)/L;
                        h00 = (2*x^3 - 3*x^2 + 1);
                        h10 = L * (x^3 - 2*x^2 + x);
                        h01 = (-2*x^3 + 3*x^2);
                        h11 = L * (x^3 - x^2);
                        if (ref == +1)
                            [~,i1] = max(nodes(left(l,:),1));
                            [~,i2] = min(nodes(left(l,:),1));
                        elseif (ref == -1)
                            [~,i2] = max(nodes(left(l,:),1));
                            [~,i1] = min(nodes(left(l,:),1));
                        end
                        ids1 = (1:6) + 6*(left(l,i1)-1);
                        ids2 = (1:6) + 6*(left(l,i2)-1);
                        LOADCASE.measurements(ids1,1) = LOADCASE.measurements(ids1,1) ...
                                                      + [0, h00, 0, 0, 0, -ref*h10]';
                        LOADCASE.measurements(ids2,1) = LOADCASE.measurements(ids2,1) ...
                                                      + [0, h01, 0, 0, 0, -ref*h11]';
                        LOADCASE.intervals(1,1) = 1;
                        break
                    end
                end
            elseif (side < 0)
                for l = 1:length(right)
                    if (any(abs(nodes(right(l,:),1) - loc) >= point(3))&& ...
                        any(abs(nodes(right(l,:),1) - loc) <= point(3)))
                        e1 = min(abs(nodes(right(l,:),1) - loc));
                        e2 = max(abs(nodes(right(l,:),1) - loc));
                        p = point(3);
                        L = e2 - e1;
                        x = (p - e1)/L;
                        h00 = (2*x^3 - 3*x^2 + 1);
                        h10 = L * (x^3 - 2*x^2 + x);
                        h01 = (-2*x^3 + 3*x^2);
                        h11 = L * (x^3 - x^2);
                        if (ref == +1)
                            [~,i1] = max(nodes(right(l,:),1));
                            [~,i2] = min(nodes(right(l,:),1));
                        elseif (ref == -1)
                            [~,i2] = max(nodes(right(l,:),1));
                            [~,i1] = min(nodes(right(l,:),1));
                        end
                        ids1 = (1:6) + 6*(right(l,i1)-1);
                        ids2 = (1:6) + 6*(right(l,i2)-1);
                        LOADCASE.measurements(ids1,1) = LOADCASE.measurements(ids1,1) ...
                                                      - [0, h00, 0, 0, 0, -ref*h10]';
                        LOADCASE.measurements(ids2,1) = LOADCASE.measurements(ids2,1) ...
                                                      - [0, h01, 0, 0, 0, -ref*h11]';
                        LOADCASE.intervals(1,1) = 1;
                        break
                    end
                end
            end

        end
        
        % =============================================================== %
        
        % Compute LOADCASE joint displacements
        function [ U ] = computeJointDisplacements(LOADCASE, stepID, C)
            % compute the final joint displacements for the assigned load case
            U = zeros(length(LOADCASE.loads(:,stepID)),1);
            U(LOADCASE.dofMap) = C\(C'\LOADCASE.loads(LOADCASE.dofMap,stepID));
        end % computeJointDisplacements
        
        % =============================================================== %
        
        % Compute LOADCASE virtual joint forces
        function [ V ] = computeVirtualDisplacements(LOADCASE, stepID, C)
            % compute the virtual joint forces for the assigned load case
            u = LOADCASE.measurements * LOADCASE.intervals(:,stepID);
            V = zeros(length(LOADCASE.loads(:,stepID)),1);
            V(LOADCASE.dofMap) = C\(C'\u(LOADCASE.dofMap));
        end % computeVirtualForces
        
        % =============================================================== %
                
        % Compute LOADCASE deflection
        function [ d ] = computeDeflection(LOADCASE, C)
            % compute the measured deflection for the assigned load case
            d = 0.0;
            for i = 1:size(LOADCASE.measurements,2)
                % zero measurement at pre-load (if specified)
                f = zeros(sum(LOADCASE.dofMap),1);
                preload = (LOADCASE.intervals(i,:) == -1);
                if any(preload)
                    f = f - LOADCASE.loads(LOADCASE.dofMap,preload);
                end
                % read measurement at end-step loading
                load = (LOADCASE.intervals(i,:) == +1);
                if any(load)
                    f = f + LOADCASE.loads(LOADCASE.dofMap,load);
                end
                % take absolute difference, and sum into aggregate
                u = LOADCASE.measurements(LOADCASE.dofMap,i)' * (C\(C'\f));
                d = d + abs(u);
            end
        end % computeDeflection
        
        % =============================================================== %
        
    end % methods
    
end

