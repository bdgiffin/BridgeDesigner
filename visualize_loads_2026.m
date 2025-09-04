% ======================================================================= %
%                   EXAMPLE BRIDGE-DESIGNER SCRIPT                        %
% ======================================================================= %
%   BridgeDesigner is a Matlab library of structural design tools,        %
%   which can be used to write scripts for the purposes of automating     %
%   various design tasks. BridgeDesigner was originally written for the   %
%   2018 NSSBC rules, and has been updated for the 2026 NSSBC rules.      %
% ======================================================================= %
clear all
clc

% Specify the name of the "dxf" drawing file
dxf = 'example_2026.dxf';

% Define the coordinates "axes" in the AutoCAD model
axes = [ '+x';   % span-direction
         '+z' ]; % up-direction

% Specify all "active" layers to be imported from the AutoCAD model
%            layer         section
active = { 'Decking',  'S-1D5000X0D065';
           'Webbing',  'R-0D1875X0D035';
           'Laterals', 'R-0D3750X0D028';
           'Chords',   'R-1D7500X0D049';
           'Tendons',  'R-0D6250X0D028';
           'Legs',     'R-2D0000X0D049' };
       
% Specify the set of all layers that comprise the "decking" support surface
decking = {'Decking'};

% Create the analysis model as a "Structure" class object
fprintf('creating model...\n')
bridge = Structure(dxf, axes, active, decking, ...
                   @defineProperties, ...
                   @computeEfficiency2026, ...
                   @defineVerticalLoadCases2026, ...
                   @defineLateralLoadCases2026);

% ======================================================================= %

% Specify the load case and step (test) IDs for visualization
display_option = "vertical";
caseID = 7;
stepID = 2;

% ======================================================================= %

% Visualize vertical/lateral loads on the bridge
if     (display_option == "vertical")
    if     (stepID == 1)
        step_name = "pre-load";
    elseif (stepID == 2)
        step_name = "backspan load";
    elseif (stepID == 3)
        step_name = "backspan+cantilever load";
    end
    fprintf('visualizing vertical load: case %i - step %i (%s)\n',caseID,stepID,step_name);
    [aggregate, d, start_step, end_step] = bridge.displayVertical(caseID,stepID);
    fprintf('aggregate deflection = %7.4f in\n', aggregate)
    fprintf('                  D1 = %7.4f in (start step: %i - end step: %i)\n', d(1), start_step(1), end_step(1))
    fprintf('                  D2 = %7.4f in (start step: %i - end step: %i)\n', d(2), start_step(2), end_step(2))
elseif (display_option == "lateral")
    if     (stepID == 1)
        test_name = "backspan";
    elseif (stepID == 2)
        test_name = "cantilever";
    end
    fprintf('visualizing lateral load: case %i (N=%i) - test %i (%s)\n',caseID,caseID+1,stepID,test_name);
    [sway] = bridge.displayLateral(caseID,stepID);
    fprintf('sway = %7.4f in\n', sway)
end
