classdef Structure < handle
    % STRUCTURE object definition
    
    % STRUCTURE (private) object properties
    properties (Access = private)
        joints
        frames
        verticalCases
        lateralCases

        % user-defined functions
        computeEfficiencyCost
        defineProperties
        
        % stiffness matrix parameters
        needStiffness
        R % reaction matrix
        K % stiffness matrix
        C % Cholesky factorization of the stiffness matrix
    end % properties
    
    % STRUCTURE (public) object methods
    methods
        
        % =============================================================== %
        
        % STRUCTURE object constructor
        function STRUCTURE = Structure(dxf, axes, active, decking, ...
                                       defineProperties, ...
                                       computeEfficiencyCost, ...
                                       defineVerticalLoadCases, ...
                                       defineLateralLoadCases)
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
            STRUCTURE.defineProperties = defineProperties;
            [ material, sections ] = STRUCTURE.defineProperties();
            STRUCTURE.frames = cell(size(frames,1),1);
            for i = 1:size(frames,1)
                frameElements = elements(frameIDs == i,:);
                sectionName = active{ismember(active(:,1),layers{groups(i)}),2};
                for j = 1:size(sections,1)
                    if (strcmp(sectionName, sections{j}.name))
                        STRUCTURE.frames{i} = Frame(STRUCTURE.joints, frames(i,:), ...
                                              frameElements, material, sections{j});
                        break
                    end
                end
            end
            
            % get all elements belonging to the "decking" layer(s)
            deckingElements = elements(ismember(groups(frameIDs),find(ismember(layers,decking))),:);

            % define all vertical and lateral load cases
            STRUCTURE.defineLoadCases(nodes, deckingElements, defineVerticalLoadCases, defineLateralLoadCases);

            % define the efficiency cost function
            STRUCTURE.computeEfficiencyCost = computeEfficiencyCost;
    
            % initialize compliance parameter
            STRUCTURE.needStiffness = true;
        end % Structure
        
        % =============================================================== %
                
        % Analyze STRUCTURE
        function [ score, sway, swayID, factor, mode, bucklingID ] = analyze(STRUCTURE)
            % run analysis of the structure
            
            % compute average structural efficiency score
            [ score ] = STRUCTURE.computeAverageEfficiency();
            
            % compute lateral deflection
            [ sway, swayID ] = STRUCTURE.computeLateralSway();

            % determine the elastic buckling load factor and mode shape
            [ factor, mode, bucklingID ] = STRUCTURE.computeBucklingLoad();
        end
        
        % =============================================================== %
        
        % Compute STRUCTURE average efficiency score
        function [ average_score ] = computeAverageEfficiency(STRUCTURE)
            % compute the average overall efficiency score of the structure ($)

            % compute the structure's efficiency scores from all load cases
            [ scores ] = STRUCTURE.computeEfficiency();
            
            % loop through all load cases and weight the average by the
            % estimated probability of occurance for each load case
            average_score = 0.0;
            for i = 1:length(STRUCTURE.verticalCases)
                average_score = average_score + STRUCTURE.verticalCases{i}.probability*scores(i);
            end

        end % computeAverageEfficiency
        
        % =============================================================== %
        
        % Compute STRUCTURE efficiency score
        function [ scores, weight, deflections, sways ] = computeEfficiency(STRUCTURE)
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
            
            % loop through all lateral load cases
            sways = zeros(size(STRUCTURE.lateralCases,1),1);
            for i = 1:size(STRUCTURE.lateralCases,1)
                sways(i) = 0.0;
                % determine the largest sway among all tests for the current case
                for j = 1:size(STRUCTURE.lateralCases,2)
                    sways(i) = max(sways(i),STRUCTURE.lateralCases{i,j}.computeDeflection(STRUCTURE.C));
                end
            end
            
            % compute efficiency scores for all cases
            [ scores ] = STRUCTURE.computeEfficiencyCost(weight, deflections, sways);
        end % computeEfficiency
        
        % =============================================================== %
                        
        % Compute STRUCTURE lateral sway
        function [ sway, caseID ] = computeLateralSway(STRUCTURE)
            % compute the worst-case lateral sway of the structure (in)
            
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();

            % loop through all lateral load cases
            sways = zeros(size(STRUCTURE.lateralCases,1),1);
            for i = 1:size(STRUCTURE.lateralCases,1)
                sways(i) = 0.0;
                % determine the largest sway among all tests for the current case
                for j = 1:size(STRUCTURE.lateralCases,2)
                    sways(i) = max(sways(i),STRUCTURE.lateralCases{i,j}.computeDeflection(STRUCTURE.C));
                end
            end
            
            % determine the worst-case lateral deflection
            [ sway, caseID ] = max(sways);
        end % computeLateralSway
        
        % =============================================================== %
                
        % Compute STRUCTURE buckling load
        function [ factor, mode, caseID ] = computeBucklingLoad(STRUCTURE)
            % compute the critical buckling load factor for the structure
            
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all vertical load cases
            factors = zeros(length(STRUCTURE.verticalCases),size(STRUCTURE.verticalCases{1}.loads,2)-1);
            modes = cell(length(STRUCTURE.verticalCases),size(STRUCTURE.verticalCases{1}.loads,2)-1);
            for i = 1:length(STRUCTURE.verticalCases)
                for jstep = 2:size(STRUCTURE.verticalCases{i}.loads,2)
                    U = STRUCTURE.verticalCases{i}.computeJointDisplacements(jstep, STRUCTURE.C);
                    G = STRUCTURE.computeGeometricStiffness(U);
                    % solve the generalized eigenvalue problem for the critical load factors
                    [ eigenvectors, D ] = eigs(STRUCTURE.K, -G, 5, 'SM');
                    eigenvalues = diag(D);
                    modes{i,jstep-1} = zeros(size(U));
                    if any(eigenvalues > 0)
                        pos = find(eigenvalues > 0);
                        [factors(i,jstep-1), loc] = min(eigenvalues(pos));
                        loc = pos(loc);
                        modes{i,jstep-1}(STRUCTURE.verticalCases{i}.dofMap) = eigenvectors(:,loc);
                    else
                        [factors(i,jstep-1), loc] = max(eigenvalues);
                        modes{i,jstep-1}(STRUCTURE.verticalCases{i}.dofMap) = eigenvectors(:,loc);
                    end
                end
            end
            
            % compute minimum buckling load factor
            if any(factors(:) > 0)
                pos = find(factors(:) > 0);
                [ factor, caseID ] = min(factors(pos));
                caseID = mod(pos(caseID)-1,length(STRUCTURE.verticalCases))+1;
                mode = modes{caseID};
            else
                [ factor, caseID ] = max(factors(:));
                caseID = mod(pos(caseID)-1,length(STRUCTURE.verticalCases))+1;
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
                for jstep = 2:size(STRUCTURE.verticalCases{i}.loads,2)
                    U = STRUCTURE.verticalCases{i}.computeJointDisplacements(jstep, STRUCTURE.C);
                    for j = 1:length(STRUCTURE.frames)
                        for k = 1:STRUCTURE.frames{j}.Nelements
                            STRUCTURE.frames{j}.elements{k}.computeCapacity(U(STRUCTURE.frames{j}.elements{k}.dofMap));
                        end
                    end
                end
            end
            
            % plot member capacities
            STRUCTURE.plotCapacities();
        end % computeMemberCapacities
        
        % =============================================================== %
                
        % Compute STRUCTURE reaction forces
        function [r] = computeReactions(STRUCTURE, caseID, stepID)
            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();
            
            % loop through all vertical load cases
            for i = 1:length(STRUCTURE.verticalCases)
                U = STRUCTURE.verticalCases{i}.computeJointDisplacements(stepID,STRUCTURE.C);
                r = STRUCTURE.R * U(STRUCTURE.verticalCases{i}.dofMap);
            end
            STRUCTURE.plotLoaded(STRUCTURE.verticalCases{caseID}.loads(:,stepID))
        end % computeReactions
        
        % =============================================================== %
        
        % Compute STRUCTURE weight
        function [ weight ] = computeWeight(STRUCTURE)
            % compute the overall weight of the structure (lbs)

            % loop over all elements in the structure
            weight = 0.0;
            for i = 1:length(STRUCTURE.frames)
                for j = 1:STRUCTURE.frames{i}.Nelements
                    weight = weight + STRUCTURE.frames{i}.elements{j}.computeWeight();
                end
            end
            
            % assume that the overall structure weight is computed based
            %   on the estimated weight of all members, plus a ~20%
            %   additional weight associated with welds and connections;
            %   call this multiplicative increase in overall weight
            %   the "weight factor":
            WF = 1.2;
            weight = WF * weight;
            
        end % computeWeight
        
        % =============================================================== %
        
        % Optimize member sections in the STRUCTURE
        function optimizeMemberSections(STRUCTURE, Niterations, FS)
            % define scoring parameters (hard-coded for NSSBC 2026)
            C_w =      75; % ($/[lb^p_w])
            p_w =     1.8;
            C_d = 4000000; % ($/in)
            
            % assume that the overall structure weight is computed based
            %   on the estimated weight of all members, plus a ~20%
            %   additional weight associated with welds and connections;
            %   call this multiplicative increase in overall weight
            %   the "weight factor":
            WF = 1.2;
            
            % get the list of all available tubing sections
            [ ~, sections ] = STRUCTURE.defineProperties();
            Nsections = length(sections);
            
            % get number of members and load cases
            Nmem = length(STRUCTURE.frames);
            Ncases = length(STRUCTURE.verticalCases);
            
            % get vector of section properties
            props = zeros(Nsections,4);
            caps  = zeros(Nsections,4);
            for i = 1:Nsections
                props(i,:) = [ 1.0/sections{i}.I, ...
                               1.0/sections{i}.J, ...
                               1.0/sections{i}.A, ...
                                   sections{i}.A ];
                caps(i,:) = [ 1.0/sections{i}.A, ...
                              sections{i}.c/sections{i}.I, ...
                              sections{i}.c/sections{i}.I, ...
                              1.0/sections{i}.I ];
            end
            
            [ ~, weight, deflections, sways ] = STRUCTURE.computeEfficiency();
            average_score = STRUCTURE.computeAverageEfficiency();
            fprintf('measured weight of structure = %6.2f lbs\n', weight)
            fprintf('maximum aggregate deflection = %7.4f in\n', max(deflections))
            import java.text.*; fmt = DecimalFormat; % (for printing comma-separated #'s)
            fprintf('average efficiency score     = $%s\n', char(fmt.format(ceil(average_score))))
            
            % perform a fixed number of iterations
            for iter = 1:Niterations
                % print current iteration number
                fprintf('iteration: %5i\n',iter)

                % compute structure compliance
                STRUCTURE.computeCompliance();

                % calculate lateral sway deflection factor gamma_lat:
                %  gamma_lat = 0.9   if  sway <= 3/8 [in]
                %  gamma_lat = 1.0   if  sway >  3/8 [in]
                gamma_lat = 1.0 - 0.1*(sways <= 0.375);

                % optimize members individually
                dS = zeros(Nmem,4);
                Smin = zeros(Nmem,4);
                for i = 1:Ncases
                    % compute structure section derivatives
                    [dSi, Smini] = STRUCTURE.computeSectionDerivatives(STRUCTURE.verticalCases{i});
                    dS = dS + STRUCTURE.verticalCases{i}.probability * gamma_lat(i) * dSi;
                    Smin = max(Smin, Smini);
                end

                % calculate sensitivity of overall score
                % score = C_w * (weight^p_w) + C_d * (gamma_lat.*deflection)
                % dscore/ddeflection = C_d * gamma_lat
                % dscore/dweight     = p_w * C_w * (weight^(p_w-1.0))
                
                % multiply by scoring parameters
                dS(:,1:3) = C_d * abs(dS(:,1:3));
                dS(:,4)   = p_w * C_w * (weight^(p_w-1.0)) * WF * dS(:,4);
                Smin = Smin * FS;

                % set new sections based on the updated derivatives
                for i = 1:Nmem
                    % choose the section that induces the least overall
                    %   cost, which does not fail under the loading
                    id = 0;
                    for j = 1:Nsections
                        if (sum(Smin(i,1:2).*caps(j,1:2)) < 1.0)&&(Smin(i,3)*caps(j,3) < 1.0)
                            if (Smin(i,4) == 0)||(Smin(i,4)*caps(j,4) < 1.0)
                                if (id == 0)
                                    id = j;
                                    cost = sum(dS(i,:) .* props(j,:));
                                elseif (sum(dS(i,:) .* props(j,:)) < cost)
                                    id = j;
                                    cost = sum(dS(i,:) .* props(j,:));
                                end
                            end
                        end
                    end
                    % if the member fails, regardless: don't change it
                    if (id ~= 0)
                        STRUCTURE.frames{i}.section.copyData(sections{id});
                    end
                end
                
                % reset the needStiffness flag to true
                STRUCTURE.needStiffness = true;
                
                [ ~, weight, deflections, sways ] = STRUCTURE.computeEfficiency();
                average_score = STRUCTURE.computeAverageEfficiency();
                fprintf('measured weight of structure = %6.2f lbs\n', weight)
                fprintf('maximum aggregate deflection = %7.4f in\n', max(deflections))
                import java.text.*; fmt = DecimalFormat; % (for printing comma-separated #'s)
                fprintf('average efficiency score     = $%s\n', char(fmt.format(ceil(average_score))))
            end
        end % optimizeMemberSections
        
        % =============================================================== % 
                
        % Display STRUCTURE vertical case/step loads and deformations
        function [aggregate,d,start_step,end_step] = displayVertical(STRUCTURE, caseID, stepID, scaling)
            arguments
                STRUCTURE    % required
                caseID       % required
                stepID       % required
                scaling = 20 % default scaling (x20)
            end

            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();

            % determine the deflection for the current vertical load case/step
            U = STRUCTURE.verticalCases{caseID}.computeJointDisplacements(stepID,STRUCTURE.C);
            [aggregate,d,start_step,end_step] = STRUCTURE.verticalCases{caseID}.computeDeflection(STRUCTURE.C);

            % display loads for the current lateral loads case/test
            STRUCTURE.plotDeformed(scaling*U)
            STRUCTURE.plotLoaded(STRUCTURE.verticalCases{caseID}.loads(:,stepID),STRUCTURE.verticalCases{caseID}.point_coordinates)
            %STRUCTURE.plotLoaded(STRUCTURE.verticalCases{caseID}.measurements(:,2))
        end % displayLateral
        
        % =============================================================== %
                
        % Display STRUCTURE lateral case/test loads and deformations
        function [sway] = displayLateral(STRUCTURE, caseID, testID, scaling)
            arguments
                STRUCTURE    % required
                caseID       % required
                testID       % required
                scaling = 20 % default scaling (x20)
            end

            % compute structure compliance, if necessary
            STRUCTURE.computeCompliance();

            % determine the sway for the current lateral load case/test
            U    = STRUCTURE.lateralCases{caseID,testID}.computeJointDisplacements(1,STRUCTURE.C);
            sway = STRUCTURE.lateralCases{caseID,testID}.computeDeflection(STRUCTURE.C);

            % display loads for the current lateral loads case/test
            STRUCTURE.plotDeformed(scaling*U)
            STRUCTURE.plotLoaded(STRUCTURE.lateralCases{caseID,testID}.loads,STRUCTURE.lateralCases{caseID,testID}.point_coordinates)
            %STRUCTURE.plotLoaded(STRUCTURE.lateralCases{caseID,testID}.measurements)
        end % displayLateral
        
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
            figure('Name','Deformed Shape')
            hold on
            % plot all elements
            for i = 1:length(STRUCTURE.frames)
                for j = 1:STRUCTURE.frames{i}.Nelements
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
            
            % set plotting options
            axis equal
            xlabel('Span')
            ylabel('Lateral')
            zlabel('Up')
            xlim([-160, +160])
            ylim([-30, +30])
            zlim([0.0, 60])
            view(0,0)
        end % plotDeformed
        
        % =============================================================== %
        
        % Plot loaded STRUCTURE
        function plotLoaded(STRUCTURE, F, points)
            arguments
                STRUCTURE   % required
                F           % required
                points = [] % deflection measurement point coordinates
            end

            % plot the loaded structure
            figure('Name','Applied Forces')
            hold on
            % plot all elements
            for i = 1:length(STRUCTURE.frames)
                for j = 1:STRUCTURE.frames{i}.Nelements
                    u = zeros(12,1);
                    STRUCTURE.frames{i}.elements{j}.plotDeformed(u);
                end
            end
            % plot all loads
            for a = 1:length(STRUCTURE.joints)
                % draw the resultant forces applied at the nodes
                quiver3(STRUCTURE.joints{a}.X(1), ...
                        STRUCTURE.joints{a}.X(2), ...
                        STRUCTURE.joints{a}.X(3), ...
                        F(1+6*(a-1)),F(2+6*(a-1)),F(3+6*(a-1)), ...
                        100,'r','LineWidth',2)
                % draw the resultant moments applied at the nodes
                quiver3(STRUCTURE.joints{a}.X(1), ...
                        STRUCTURE.joints{a}.X(2), ...
                        STRUCTURE.joints{a}.X(3), ...
                        F(4+6*(a-1)),F(5+6*(a-1)),F(6+6*(a-1)), ...
                        100,'m','LineWidth',2)
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
            % plot all deflection measurement point coordinates
            for a = 1:size(points,2)
                plot3(points(1,a), ...
                      points(2,a), ...
                      points(3,a), ...
                      'oc','LineWidth',4,'MarkerSize',10)
            end
            
            % set plotting options
            axis equal
            xlabel('Span')
            ylabel('Lateral')
            zlabel('Up')
            xlim([-160, +160])
            ylim([-30, +30])
            zlim([0.0, 60])
            view(0,0)
        end % plotLoaded
        
        % =============================================================== %
                
        % Plot capacities of all members in the STRUCTURE
        function plotCapacities(STRUCTURE)
            % plot member capacities
            figure('Name','Member Capacities')
            hold on
            % plot all elements
            for i = 1:length(STRUCTURE.frames)
                for j = 1:STRUCTURE.frames{i}.Nelements
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
            
            % set plotting options
            axis equal
            xlabel('Span')
            ylabel('Lateral')
            zlabel('Up')
            xlim([-160, +160])
            ylim([-30, +30])
            zlim([0.0, 60])
            view(0,0)
            cmap = [min(max((-2.0:0.01:2.0),0.0),1.0); ...
                    min(2.0-abs(-2.0:0.01:2.0),1.0); ...
                    min(max((2.0:-0.01:-2.0),0.0),1.0)]';
            colormap(cmap)
            cbar = colorbar;
            ylabel(cbar,'Demand/Capacity ratio')
            caxis([0.0,1.0])
        end % plotCapacities
        
        % =============================================================== %
                    
        % Output STRUCTURE drawing data to a .dxf file
        function writeDXF(STRUCTURE, dxf)
            % create a .dxf file, and write the frame member data (sections)
            
            % define the AutoCAD color remapping scheme
            colorRemap = [(10:10:240), (11:10:241), ...
                          (12:10:242), (13:10:243), ...
                          (14:10:244), (15:10:245), ...
                          (16:10:246), (17:10:247), ...
                          (18:10:248), (19:10:249)];
            
            % open dxf file
            fid = fopen(dxf, 'w');
            
            % print the header section info
            fprintf(fid, '999\n');
            fprintf(fid, 'File generated by BridgeDesigner\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'SECTION\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'HEADER\n');
            fprintf(fid, '9\n');
            fprintf(fid, '$ACADVER\n');
            fprintf(fid, '1\n');
            fprintf(fid, 'AC1006\n');
            fprintf(fid, '9\n');
            fprintf(fid, '$INSBASE\n');
            fprintf(fid, '10\n');
            fprintf(fid, '0\n');
            fprintf(fid, '20\n');
            fprintf(fid, '0\n');
            fprintf(fid, '30\n');
            fprintf(fid, '0\n');
            
            % compute and print drawing extents
            xmin = STRUCTURE.joints{1}.X;
            xmax = STRUCTURE.joints{1}.X;
            for a = 2:length(STRUCTURE.joints)
                xmin = min(STRUCTURE.joints{a}.X, xmin);
                xmax = max(STRUCTURE.joints{a}.X, xmax);
            end
            fprintf(fid, '9\n');
            fprintf(fid, '$EXTMIN\n');
            fprintf(fid, '10\n');
            fprintf(fid, '%f\n',xmin(1));
            fprintf(fid, '20\n');
            fprintf(fid, '%f\n',xmin(2));
            fprintf(fid, '30\n');
            fprintf(fid, '%f\n',xmin(3));
            fprintf(fid, '9\n');
            fprintf(fid, '$EXTMAX\n');
            fprintf(fid, '10\n');
            fprintf(fid, '%f\n',xmax(1));
            fprintf(fid, '20\n');
            fprintf(fid, '%f\n',xmax(2));
            fprintf(fid, '30\n');
            fprintf(fid, '%f\n',xmax(3));
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDSEC\n');
            
            % print the drawing table data
            fprintf(fid, '0\n');
            fprintf(fid, 'SECTION\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'TABLES\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'TABLE\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'LTYPE\n');
            fprintf(fid, '70\n');
            fprintf(fid, '1\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'LTYPE\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'Continuous\n');
            fprintf(fid, '70\n');
            fprintf(fid, '64\n');
            fprintf(fid, '3\n');
            fprintf(fid, 'Solid line\n');
            fprintf(fid, '72\n');
            fprintf(fid, '65\n');
            fprintf(fid, '73\n');
            fprintf(fid, '0\n');
            fprintf(fid, '40\n');
            fprintf(fid, '0\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDTAB\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'TABLE\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'LAYER\n');
            fprintf(fid, '70\n');
            fprintf(fid, '1000\n');
            
            % get the list of all available tubing sections
            [ ~, sections ] = STRUCTURE.defineProperties();
            Nsections = length(sections);
            sectionNames = cell(Nsections,1);
            for i = 1:Nsections
                sectionNames{i} = sections{i}.name;
            end
            
            % determine the number of drawing layers to export,
            %   based on the assigned tubing section sizes
            layers = false(Nsections,1);
            for i = 1:length(STRUCTURE.frames)
                layers = (layers|ismember(sectionNames,STRUCTURE.frames{i}.section.name));
            end
            layers = find(layers);
            Nlayers = length(layers);
            for i = 1:Nlayers
                fprintf(fid, '0\n');
                fprintf(fid, 'LAYER\n');
                fprintf(fid, '2\n');
                fprintf(fid, '%s\n',sectionNames{layers(i)});
                %fprintf(fid, '%i\n',layers(i));
                fprintf(fid, '70\n');
                fprintf(fid, '64\n');
                fprintf(fid, '62\n');
                fprintf(fid, '%i\n',colorRemap(layers(i)));
                fprintf(fid, '6\n');
                fprintf(fid, 'Continuous\n');
            end
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDTAB\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'TABLE\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'STYLE\n');
            fprintf(fid, '70\n');
            fprintf(fid, '0\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDTAB\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDSEC\n');
            
            % print block data
            fprintf(fid, '0\n');
            fprintf(fid, 'SECTION\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'BLOCKS\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDSEC\n');
            
            % print all drawing entities (lines)
            fprintf(fid, '0\n');
            fprintf(fid, 'SECTION\n');
            fprintf(fid, '2\n');
            fprintf(fid, 'ENTITIES\n');
            for i = 1:length(STRUCTURE.frames)
                fprintf(fid, '0\n');
                fprintf(fid, 'LINE\n');
                fprintf(fid, '8\n');
                fprintf(fid, '%s\n',STRUCTURE.frames{i}.section.name);
                %fprintf(fid, '%i\n',find(ismember(sectionNames,STRUCTURE.frames{i}.section.name)));
                fprintf(fid, '10\n');
                fprintf(fid, '%f\n',STRUCTURE.frames{i}.joints{1}.X(1));
                fprintf(fid, '20\n');
                fprintf(fid, '%f\n',STRUCTURE.frames{i}.joints{1}.X(2));
                fprintf(fid, '30\n');
                fprintf(fid, '%f\n',STRUCTURE.frames{i}.joints{1}.X(3));
                fprintf(fid, '11\n');
                fprintf(fid, '%f\n',STRUCTURE.frames{i}.joints{2}.X(1));
                fprintf(fid, '21\n');
                fprintf(fid, '%f\n',STRUCTURE.frames{i}.joints{2}.X(2));
                fprintf(fid, '31\n');
                fprintf(fid, '%f\n',STRUCTURE.frames{i}.joints{2}.X(3));
            end
            fprintf(fid, '0\n');
            fprintf(fid, 'ENDSEC\n');
            fprintf(fid, '0\n');
            fprintf(fid, 'EOF');
            
            % close dxf file
            fclose(fid);
            
        end % writeDXF
        
        % =============================================================== %
        
    end % (public) methods
    
    % STRUCTURE (private) object methods
    methods (Access = private)
        
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
        function defineLoadCases(STRUCTURE, nodes, decking, defineVerticalLoadCases, defineLateralLoadCases)
            % create dofMap
            dofMap = false(6*length(STRUCTURE.joints),1);
            for a = 1:length(STRUCTURE.joints)
                dofMap((1:6)+6*(a-1)) = STRUCTURE.joints{a}.free;
            end
            
            % create vertical load cases
            [ loads, measurements, probabilities ] = defineVerticalLoadCases();
            STRUCTURE.verticalCases = cell(size(loads,1),1);
            for i = 1:size(loads,1)
                STRUCTURE.verticalCases{i} = LoadCase(size(nodes,1), size(loads,3), size(measurements,2), dofMap, probabilities(i));
                STRUCTURE.verticalCases{i}.assignVerticalLoads(loads(i,:,:), nodes, decking)
                STRUCTURE.verticalCases{i}.assignVerticalMeasurements(measurements(i,:), nodes, decking)
            end
            
            % create lateral load cases
            [ loads, measurements, probabilities ] = defineLateralLoadCases();
            STRUCTURE.lateralCases = cell(size(loads));
            for i = 1:size(loads,1)
                for j = 1:size(loads,2)
                    STRUCTURE.lateralCases{i,j} = LoadCase(size(nodes,1), 1, 1, dofMap, probabilities(i));
                    STRUCTURE.lateralCases{i,j}.assignLateralLoad(loads{i,j}, nodes, decking)
                    STRUCTURE.lateralCases{i,j}.assignLateralMeasurement(measurements{i,j}, nodes, decking)
                end
            end
            
        end % defineLoadCases
        
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
                    for j = 1:STRUCTURE.frames{i}.Nelements
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
                for j = 1:STRUCTURE.frames{i}.Nelements
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
        
        % Compute STRUCTURE section cost derivatives
        function [dS, Smin] = computeSectionDerivatives(STRUCTURE, loadCase)
            % assemble the section derivatives for the entire STRUCTURE
            % dS(i,1) := d(cost)/dIi^-1
            % dS(i,2) := d(cost)/dJi^-1
            % dS(i,3) := d(cost)/dAi^-1
            % dS(i,4) := d(cost)/dAi
            % Smin(i,1) := d(axial+bending)/dA^-1
            % Smin(i,2) := d(axial+bending)/dS^-1
            % Smin(i,3) := d(shear)/dS^-1
            % Smin(i,4) := d(buckling)/dI^-1
            
            % count up all degrees of freedom
            Ndof = 0;
            for a = 1:length(STRUCTURE.joints)
                Ndof = Ndof + sum(STRUCTURE.joints{a}.dofs > 0);
            end
            
            % initialize the section derivatives matrix
            Nmem = length(STRUCTURE.frames);
            dS = zeros(Nmem,4);
            Smin = zeros(Nmem,4);

            for jstep = 1:size(loadCase.loads,2)
                % compute the joint displacement and inverse measurement vectors
                U = loadCase.computeJointDisplacements(jstep, STRUCTURE.C);
                V = loadCase.computeVirtualDisplacements(jstep, STRUCTURE.C);

                % loop through all elements, and assemble stiffness contributions
                for i = 1:length(STRUCTURE.frames)
                    for j = 1:STRUCTURE.frames{i}.Nelements
                        dofMap = STRUCTURE.frames{i}.elements{j}.dofMap;
                        [ds, smin] = STRUCTURE.frames{i}.elements{j}.computeStiffnessDerivatives(U(dofMap), V(dofMap));
                        dS(i,:) = dS(i,:) + ds;
                        Smin(i,:) = max(Smin(i,:), smin);
                    end
                end
            end
            
            % loop through all elements, and assemble weight contributions
            for i = 1:length(STRUCTURE.frames)
                for j = 1:STRUCTURE.frames{i}.Nelements
                    [ds] = STRUCTURE.frames{i}.elements{j}.computeWeightDerivatives();
                    dS(i,:) = dS(i,:) + ds;
                end
            end
            
        end % computeSectionDerivatives
                
        % =============================================================== %
        
    end % (private) methods
    
end

