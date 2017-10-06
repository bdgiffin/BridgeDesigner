classdef Structure < handle
    % STRUCTURE object definition
    
    % STRUCTURE object properties
    properties (SetAccess = private)
        joints
        frames
        verticalCases
        lateralCases
        
        % stiffness matrix parameters
        needStiffness
        R % reaction matrix
        K % stiffness matrix
        C % compliance matrix (cholesky factor of stiffness)
        
    end % properties
    
    % STRUCTURE object methods
    methods
        
        % =============================================================== %
        
        % STRUCTURE object constructor
        function STRUCTURE = Structure(dxf, axes, active)
            % read the input .dxf drawing file
            [ nodes, frames, groups, layers ] = STRUCTURE.readDXF(dxf);
            
            % select all active layers
            [ nodes, frames, groups, layers ] = STRUCTURE.selectActiveLayers( ...
              nodes, frames, groups, layers, active);
            
            % transform the nodal coordinates
            [ nodes ] = STRUCTURE.transformCoordinates(axes, nodes);
          
            % mesh all frames into sub-elements
            [ elements, frameIDs ] = STRUCTURE.meshElements(nodes, frames);
            
            % define all joints in the STRUCTURE
            Ndof = 0;
            Nsup = 0;
            STRUCTURE.joints = cell(size(nodes,1),1);
            for a = 1:size(nodes,1)
                dofs = zeros(6,1);
                if (nodes(a,3) < 1.0e-6)
                    dofs(1:3) = -((1:3) + Nsup);
                    Nsup = Nsup + 3;
                    dofs(4:6) = (1:3) + Ndof;
                    Ndof = Ndof + 3;
                else
                    dofs(1:6) = (1:6) + Ndof;
                    Ndof = Ndof + 6;
                end
                STRUCTURE.joints{a} = Joint(nodes(a,:), a, dofs);
            end
            
            % define all frame objects in the STRUCTURE
            [ material, sections ] = defineProperties();
            STRUCTURE.frames = cell(size(frames,1),1);
            for i = 1:size(frames,1)
                frameElements = elements(frameIDs == i,:);
                sectionName = active{ismember(active(:,1),layers{groups(i)}),2};
                for j = 1:size(sections,1)
                    if (strcmp(sectionName, sections{j}.name))
                        STRUCTURE.frames{i} = Frame(STRUCTURE.joints, frameElements, ...
                                              material, sections{j});
                        break
                    end
                end
            end
            
            % get all elements belonging to the "Decking" layer
            decking = elements(groups(frameIDs) == find(ismember(layers,'Decking')),:);

            % define all vertical and lateral load cases
            STRUCTURE.defineLoadCases(nodes, decking);
    
            % initialize compliance parameter
            STRUCTURE.needStiffness = true;
        end % Structure
        
        % =============================================================== %
        
        % Read STRUCTURE geometry from the specified .dxf file
        function [ nodes, elements, groups, layers ] = readDXF(STRUCTURE, dxf)
            %  Converts a plain AutoCAD .dxf (2013) file into nodes
            %  and elements, and stores additional layer data
            %  dxf: must specify a .dxf file

            % open the .dxf file
            fileID = fopen(dxf);

            % extract all AutoCAD lines found in the model
            EOF = [];
            Nlines = 0;
            lines = [];
            Nlayers = 0;
            layers = cell(1,1);
            groups = [];
            while isempty(EOF)
                string1 = fgets(fileID);
                if (strcmp(strtrim(string1),'AcDbLayerTableRecord'))
                    % get the current layer's name
                    string2 = fgets(fileID);
                    Nlayers = Nlayers + 1;
                    layers{Nlayers} = strtrim(fgets(fileID));
                elseif (strcmp(strtrim(string1),'LINE'))
                    % find the AcDbEntity entry
                    string2 = fgets(fileID);
                    while (~strcmp(strtrim(string2),'AcDbEntity'))
                        string2 = fgets(fileID);
                    end
                    string2 = fgets(fileID);
                    % get the current line's layer (group) ID
                    string2 = strtrim(fgets(fileID));
                    for i = 1:Nlayers
                        if strcmp(string2,layers{i})
                            groups = [groups; i];
                        end
                    end
                    % find the AcDbLine entry
                    string2 = fgets(fileID);
                    while (~strcmp(strtrim(string2),'AcDbLine'))
                        string2 = fgets(fileID);
                    end
                    % read the current line's coordinates
                    lines = [lines; zeros(1,6)];
                    Nlines = Nlines + 1;
                    for i = 1:6
                        string2 = fgets(fileID);
                        lines(Nlines,i) = str2double(fgets(fileID));
                    end
                end
                EOF = strfind(string1,'EOF');
            end

            % close the AutoCAD .dxf file
            fclose(fileID);

            % construct a list of unique nodes
            elements = zeros(Nlines,2);
            elements(1,:) = [1, 2];
            nodes = [lines(1,1:3); lines(1,4:6)];
            Nnodes = 2;
            for i = 2:Nlines
                for j = 1:2
                    %compare the current coordinates with all unique nodes
                    coordinates = lines(i,((3*(j-1)+1):(3*(j-1)+3)));
                    diff = sum(((nodes - repmat(coordinates,Nnodes,1)).^2),2);
                    if (any(diff < 1.0e-6))
                        % coordinate is not unique
                        elements(i,j) = find(diff < 1.0e-6);
                    else
                        % coordinate is a unique new node
                        nodes = [nodes; coordinates];
                        Nnodes = Nnodes + 1;
                        elements(i,j) = Nnodes;
                    end
                end
            end

        end % readDXF
        
        % =============================================================== % 
        
        % Transform STRUCTURE coordinates
        function [ nodes ] = transformCoordinates(STRUCTURE, axes, nodes)
            % determine the "span" and "up" coordinate directions in the model
            span = axes(1,:);
            up   = axes(2,:);

            % convert the "span" coordinate into a vector
            span_dir = zeros(3,1);
            if     (span(2) == 'x'); span_dir(1) = 1;
            elseif (span(2) == 'y'); span_dir(2) = 1;
            elseif (span(2) == 'z'); span_dir(3) = 1;
            end
            if (span(1) == '-'); span_dir = - span_dir; end

            % convert the "up" coordinate into a vector
            up_dir = zeros(3,1);
            if     (up(2) == 'x'); up_dir(1) = 1;
            elseif (up(2) == 'y'); up_dir(2) = 1;
            elseif (up(2) == 'z'); up_dir(3) = 1;
            end
            if (up(1) == '-'); up_dir = - up_dir; end

            % compute the "lateral" coordinate from the cross-product of (up x span)
            lateral_dir = cross(up_dir, span_dir);
            
            % definte a permutation matrix P to transform the nodal coordinates
            % such that: span = +x, lateral = +y, up = +z
            P = [span_dir, lateral_dir, up_dir];
            nodes = nodes * P;
            
            % shift the nodal coordinates appropriately
            nodes(:,1) = nodes(:,1) - mean(nodes(:,1));
            nodes(:,2) = nodes(:,2) - mean(nodes(:,2));
            nodes(:,3) = nodes(:,3) - min(nodes(:,3));
        end % transformCoordinates
        
        % =============================================================== %

        % Select active layers in the STRUCTURE
        function [ nodes, elements, groups, layers ] = selectActiveLayers( ...
                   STRUCTURE, nodes, elements, groups, layers, active)
            % read the active layers
            Nactive = size(active,1);
            activeID  = zeros(Nactive,1);
            sectionID = zeros(Nactive,1);
            for i = 1:Nactive
                % get the active layer ID
                for j = 1:length(layers)
                    if strcmp(active{i,1},layers{j})
                        activeID(i) = j;
                    end
                end
            end

            % remove any elements belonging to inactive layers from the model
            active_elements = false(size(elements,1),1);
            for i = 1:Nactive
                active_elements(groups == activeID(i)) = true;
            end
            elements = elements(active_elements,:);
            groups = groups(active_elements);

            % remove any nodes belonging to inactive layers from the model
            active_nodes = false(size(nodes,1),1);
            for i = 1:size(elements,1)
                active_nodes(elements(i,:)) = true;
            end
            new_nodeID = zeros(size(nodes,1),1);
            Nnodes = 0;
            for i = 1:size(nodes,1)
                if active_nodes(i)
                    Nnodes = Nnodes + 1;
                    new_nodeID(i) = Nnodes;
                end
            end
            nodes = nodes(active_nodes,:);
            for i = 1:size(elements,1)
                elements(i,:) = new_nodeID(elements(i,:));
            end
        end % selectActiveLayers
        
        % =============================================================== %
        
        % Mesh all elements in the STRUCTURE
        function [ elements, frameIDs ] = meshElements(STRUCTURE, nodes, elements)
            % loop over all frames, and break any intersecting lines
            i = 0;
            frameIDs = (1:size(elements,1));
            while(i < size(elements,1))
                i = i + 1;
                a = nodes(elements(i,1),:);
                d = nodes(elements(i,2),:) - a;
                L = norm(d,2);
                d = d / L;
                proximal = find((abs(nodes(:,1) - a(1)) < L)& ...
                                (abs(nodes(:,2) - a(2)) < L)& ...
                                (abs(nodes(:,3) - a(3)) < L));
                for j = 1:length(proximal)
                    if ~any(elements(i,:) == proximal(j))
                        b = nodes(proximal(j),:) - a;
                        s = d * b';
                        if ((norm(b - s*d,2) < 1.0e-6)&&(s > 0)&&(s < L))
                            elements = [elements; proximal(j), elements(i,2)];
                            frameIDs = [frameIDs, frameIDs(i)];
                            elements(i,2) = proximal(j);
                            L = s;
                        end
                    end
                end
            end
        end % meshElements
        
        % =============================================================== %
          
        % Define all vertical and lateral load cases on the STRUCTURE
        function defineLoadCases(STRUCTURE, nodes, decking)
            % create dofMap
            dofMap = false(6*length(STRUCTURE.joints),1);
            for a = 1:length(STRUCTURE.joints)
                dofMap((1:6)+6*(a-1)) = STRUCTURE.joints{a}.free;
            end
            
            % create vertical load cases
            [ loads, measurements ] = defineVerticalLoadCases();
            STRUCTURE.verticalCases = cell(size(loads,1),1);
            for i = 1:size(loads,1)
                STRUCTURE.verticalCases{i} = LoadCase(size(nodes,1), size(loads,2), size(measurements,2), dofMap);
                STRUCTURE.verticalCases{i}.assignVerticalLoads(loads(i,:,:), nodes, decking)
                STRUCTURE.verticalCases{i}.assignVerticalMeasurements(measurements(i,:), nodes, decking)
            end
            
            % create lateral load cases
            [ loads, measurements ] = defineLateralLoadCases();
            STRUCTURE.lateralCases = cell(size(loads,1),1);
            for i = 1:size(loads,1)
                STRUCTURE.lateralCases{i} = LoadCase(size(nodes,1), 1, 1, dofMap);
                STRUCTURE.lateralCases{i}.assignLateralLoad(loads{i}, nodes, decking)
                STRUCTURE.lateralCases{i}.assignLateralMeasurement(measurements{i}, nodes, decking)
            end
            
        end % defineLoadCases
        
        % =============================================================== %
                
        % Analyze STRUCTURE
        function [ score, sway, swayID, factor, mode, bucklingID ] = analyze(STRUCTURE)
            % run analysis of the structure
            
            % compute structural efficiency score
            [ score ] = STRUCTURE.computeEfficiency();
            
            % compute lateral deflection
            [ sway, swayID ] = STRUCTURE.computeLateralSway();

            % determine the elastic buckling load factor and mode shape
            [ factor, mode, bucklingID ] = STRUCTURE.computeBucklingLoad();
        end
        
        % =============================================================== %
        
        % Compute STRUCTURE efficiency score
        function [ score, weight, deflections ] = computeEfficiency(STRUCTURE)
            % compute the overall efficiency score of the structure ($)

            % compute the structure's overall weight
            weight = STRUCTURE.computeWeight();
            
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all vertical load cases
            deflections = zeros(length(STRUCTURE.verticalCases),1);
            for i = 1:length(STRUCTURE.verticalCases)
                deflections(i) = STRUCTURE.verticalCases{i}.computeDeflection(STRUCTURE.C);
            end
            
            % compute efficiency score
            [ score ] = computeEfficiency(weight, mean(deflections));
        end % computeEfficiency
        
        % =============================================================== %
                        
        % Compute STRUCTURE lateral sway
        function [ sway, caseID ] = computeLateralSway(STRUCTURE)
            % compute the worst-case lateral sway of the structure (in)
            
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all lateral load cases
            deflection = zeros(length(STRUCTURE.lateralCases),1);
            for i = 1:length(STRUCTURE.lateralCases)
                deflection(i) = STRUCTURE.lateralCases{i}.computeDeflection(STRUCTURE.C);
            end
            
            % determine the worst-case lateral deflection
            [ sway, caseID ] = max(deflection);
        end % computeLateralSway
        
        % =============================================================== %
                
        % Compute STRUCTURE buckling load
        function [ factor, mode, caseID ] = computeBucklingLoad(STRUCTURE)
            % compute the critical buckling load factor for the structure
            
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all vertical load cases
            factors = zeros(length(STRUCTURE.verticalCases),1);
            modes = cell(length(STRUCTURE.verticalCases),1);
            for i = 1:length(STRUCTURE.verticalCases)
                U = STRUCTURE.verticalCases{i}.computeJointDisplacements(STRUCTURE.C);
                G = STRUCTURE.computeGeometricStiffness(U);
                % solve the generalized eigenvalue problem for the critical load factors
                [ eigenvectors, D ] = eigs(STRUCTURE.K, -G, 5, 'SM');
                eigenvalues = diag(D);
                modes{i} = zeros(size(U));
                if any(eigenvalues > 0)
                    pos = find(eigenvalues > 0);
                    [factors(i), loc] = min(eigenvalues(pos));
                    loc = pos(loc);
                    modes{i}(STRUCTURE.verticalCases{i}.dofMap) = eigenvectors(:,loc);
                else
                    [factors(i), loc] = max(eigenvalues);
                    modes{i}(STRUCTURE.verticalCases{i}.dofMap) = eigenvectors(:,loc);
                end
            end
            
            % compute minimum buckling load factor
            if any(factors > 0)
                pos = find(factors > 0);
                [ factor, caseID ] = min(factors(pos));
                caseID = pos(caseID);
                mode = modes{caseID};
            else
                [ factor, caseID ] = max(factors);
                mode = modes{caseID};
            end
        end % computeBucklingLoad
        
        % =============================================================== %
        
        % Compute STRUCTURE member capacities
        function computeMemberCapacities(STRUCTURE)
            % compute the worst-case capacities of all members
            
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all vertical load cases
            for i = 1:length(STRUCTURE.verticalCases)
                U = STRUCTURE.verticalCases{i}.computeJointDisplacements(STRUCTURE.C);
                for j = 1:length(STRUCTURE.frames)
                    for k = 1:length(STRUCTURE.frames{j}.elements)
                        u = U(STRUCTURE.frames{j}.elements{k}.dofMap);
                        STRUCTURE.frames{j}.elements{k}.computeCapacity(u);
                    end
                end
            end
            
            % plot member capacities
            STRUCTURE.plotCapacities();
        end % computeBucklingLoad
        
        % =============================================================== %
                
        % Compute STRUCTURE reaction forces
        function computeReactions(STRUCTURE)
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all vertical load cases
            for i = 1:length(STRUCTURE.verticalCases)
                U = STRUCTURE.verticalCases{i}.computeJointDisplacements(STRUCTURE.C);
                r = STRUCTURE.R * U(STRUCTURE.verticalCases{i}.dofMap);
            end
            STRUCTURE.plotLoaded(STRUCTURE.verticalCases{1}.loads(:,end))
        end % computeReactions
        
        % =============================================================== %
        
        % Compute STRUCTURE weight
        function [ weight ] = computeWeight(STRUCTURE)
            % compute the overall weight of the structure (lbs)

            % loop over all elements in the structure
            weight = 0.0;
            for i = 1:length(STRUCTURE.frames)
                for j = 1:length(STRUCTURE.frames{i}.elements)
                    weight = weight + STRUCTURE.frames{i}.elements{j}.computeWeight();
                end
            end
        end % computeWeight
        
        % =============================================================== %
        
        % Compute STRUCTURE compliance matrix (inverse stiffness)
        function computeCompliance(STRUCTURE)
            %  compute the compliance matrix for the entire structure

            if (STRUCTURE.needStiffness)
                % count up all degrees of freedom
                Ndof = 0;
                for a = 1:length(STRUCTURE.joints)
                    Ndof = Ndof + sum(STRUCTURE.joints{a}.dofs > 0);
                end

                % initialize the stiffness (and joint reaction) matricies
                STRUCTURE.K = sparse(Ndof, Ndof);
                STRUCTURE.R = sparse(6 * length(STRUCTURE.joints) - Ndof, Ndof);

                % loop through all elements, and assemble contributions
                for i = 1:length(STRUCTURE.frames)
                    for j = 1:length(STRUCTURE.frames{i}.elements)
                        ids = STRUCTURE.frames{i}.elements{j}.free;
                        map = STRUCTURE.frames{i}.elements{j}.freeDofs;
                        sup = STRUCTURE.frames{i}.elements{j}.fixedDofs;
                        k = STRUCTURE.frames{i}.elements{j}.computeStiffness();
                        if ~isempty(map)
                            STRUCTURE.K(map,map) = STRUCTURE.K(map,map) + k(ids,ids);
                            if ~isempty(sup)
                                STRUCTURE.R(sup,map) = STRUCTURE.R(sup,map) + k(~ids,ids);
                            end
                        end
                    end
                end
            
                % factor the global stiffness matrix to get compliance
                STRUCTURE.C = chol(STRUCTURE.K);
                STRUCTURE.needStiffness = false;
            end
        end % computeCompliance
        
        % =============================================================== %
        
        % Compute STRUCTURE geometric stiffness matrix
        function [G] = computeGeometricStiffness(STRUCTURE, U)
            % assemble the global geometric stiffness matrix for the entire structure
            
            % initialize the geometric stiffness matrix
            G = 0.0 * STRUCTURE.K;
            
            % loop through all elements, and assemble contributions
            for i = 1:length(STRUCTURE.frames)
                for j = 1:length(STRUCTURE.frames{i}.elements)
                    ids = STRUCTURE.frames{i}.elements{j}.free;
                    map = STRUCTURE.frames{i}.elements{j}.freeDofs;
                    u = U(STRUCTURE.frames{i}.elements{j}.dofMap);
                    g = STRUCTURE.frames{i}.elements{j}.computeGeometricStiffness(u);
                    if ~isempty(map)
                        G(map,map) = G(map,map) + g(ids,ids);
                    end
                end
            end
        end % computeGeometricStiffness
        
        % =============================================================== %
                
        % Plot undeformed STRUCTURE
        function plotUndeformed(STRUCTURE)
            % plot the undeformed structure
            U = zeros(6*length(STRUCTURE.joints),1);
            STRUCTURE.plotDeformed(U)
        end % plotUndeformed
        
        % =============================================================== %
        
        % Plot deformed STRUCTURE
        function plotDeformed(STRUCTURE, U)
            % plot the deformed structure
            figure(1)
            hold on
            % plot all elements
            for i = 1:length(STRUCTURE.frames)
                for j = 1:length(STRUCTURE.frames{i}.elements)
                    u = U(STRUCTURE.frames{i}.elements{j}.dofMap);
                    STRUCTURE.frames{i}.elements{j}.plotDeformed(u);
                end
            end
            % plot all supports
            for a = 1:length(STRUCTURE.joints)
                if (sum(STRUCTURE.joints{a}.dofs < 0) == 3)
                    plot3(STRUCTURE.joints{a}.X(1), ...
                          STRUCTURE.joints{a}.X(2), ...
                          STRUCTURE.joints{a}.X(3), ...
                          '^g')
                end
            end
            axis equal
        end % plotDeformed
        
        % =============================================================== %
        
        % Plot deformed STRUCTURE
        function plotLoaded(STRUCTURE, F)
            % plot the loaded structure
            figure(2)
            hold on
            % plot all elements
            for i = 1:length(STRUCTURE.frames)
                for j = 1:length(STRUCTURE.frames{i}.elements)
                    u = zeros(12,1);
                    STRUCTURE.frames{i}.elements{j}.plotDeformed(u);
                end
            end
            % plot all loads
            for a = 1:length(STRUCTURE.joints)
                quiver3(STRUCTURE.joints{a}.X(1), ...
                        STRUCTURE.joints{a}.X(2), ...
                        STRUCTURE.joints{a}.X(3), ...
                        F(1+6*(a-1)),F(2+6*(a-1)),F(3+6*(a-1)), ...
                        100,'g')
                quiver3(STRUCTURE.joints{a}.X(1), ...
                        STRUCTURE.joints{a}.X(2), ...
                        STRUCTURE.joints{a}.X(3), ...
                        F(4+6*(a-1)),F(5+6*(a-1)),F(6+6*(a-1)), ...
                        100,'r')
            end
            % plot all supports
            for a = 1:length(STRUCTURE.joints)
                if (sum(STRUCTURE.joints{a}.dofs < 0) == 3)
                    plot3(STRUCTURE.joints{a}.X(1), ...
                          STRUCTURE.joints{a}.X(2), ...
                          STRUCTURE.joints{a}.X(3), ...
                          '^g')
                end
            end
            axis equal
        end % plotLoaded
        
        % =============================================================== %
                
        % Plot capacities of all members in the STRUCTURE
        function plotCapacities(STRUCTURE)
            % plot member capacities
            figure(3)
            hold on
            % plot all elements
            for i = 1:length(STRUCTURE.frames)
                for j = 1:length(STRUCTURE.frames{i}.elements)
                    STRUCTURE.frames{i}.elements{j}.plotCapacity();
                end
            end
            % plot all supports
            for a = 1:length(STRUCTURE.joints)
                if (sum(STRUCTURE.joints{a}.dofs < 0) == 3)
                    plot3(STRUCTURE.joints{a}.X(1), ...
                          STRUCTURE.joints{a}.X(2), ...
                          STRUCTURE.joints{a}.X(3), ...
                          '^g')
                end
            end
            axis equal
        end % plotCapacities
        
        % =============================================================== %
        
    end % methods
    
end

