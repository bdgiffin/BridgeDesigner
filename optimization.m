% ======================================================================= %
%             EXAMPLE BRIDGE-DESIGNER OPTIMIZATION SCRIPT                 %
% ======================================================================= %
%   BridgeDesigner is a Matlab library of structural design tools,        %
%   which can be used to write scripts for the purposes of automating     %
%   various design tasks. BridgeDesigner was originally written for the   %
%   2018 NSSBC rules, and has been updated for the 2026 NSSBC rules.      %
% ======================================================================= %
clear all
clc

% Specify the name of the "dxf" drawing file
%   - .dwg files will not work! Files must be saved in .dxf format
%   - if the file is saved locally within the BridgeBuilder folder,
%     then you need only specify the name without the directory,
%     e.g.
%            
%     dxf = 'example.dxf';
%
%     otherwise, if you are attempting to reference a file stored
%     in another folder, you will need to specify either the
%     file's absolute path, e.g.
%
%     dxf = 'C:\Path\To\File\example.dxf';
%
%     or it's path relative to the BridgeBuilder directory, e.g.
%
%     dxf = '..\..\Relative\Path\example.dxf';
%
dxf = 'example.dxf';

% Define the coordinates "axes" in the AutoCAD model
%   - Span-direction indicates the coordinate axis aligned with the path
%     of travel moving across the bridge
%   - Up-direction indicates the coordinate axis aligned with the vertical
%     direction (the opposite of the gravity direction)
%   Acceptable arguments are: '+x', '-x', '+y', '-y', '+z', '-z'
axes = [ '+x';   % span-direction
         '+y' ]; % up-direction

% Specify all "active" layers to be imported from the AutoCAD model,
%   and identify which tubing section should be assigned to each. 
%   - Use the layer naming convention as it appears in the AutoCAD model
%   - Refer to the "defineProperties.m" file regarding the naming
%     convention for different tubing sections
%            layer         section
active = { 'Decking',  'S-1D0000X0D035';
           'Webbing',  'R-0D1875X0D035';
           'Laterals', 'R-0D5000X0D035';
           'Chords',   'R-1D2500X0D035';
           'Tendons',  'R-0D5000X0D035';
           'Legs',     'R-1D0000X0D028' };
       
% Specify the set of all layers that comprise the "decking" support surface
%   - Specify decking as a cell array, containing the string names of
%     all decking layers which are also in the "active" layer set
decking = {'Decking'};

% Create the analysis model as a "Structure" class object, indicating
%   the "dxf" file, coordinate "axes", and "active" layers,
%   defined previously.
fprintf('creating model...\n')
bridge = Structure(dxf, axes, active, decking, ...
                   @defineProperties, ...
                   @computeEfficiency2026, ...
                   @defineVerticalLoadCases2026, ...
                   @defineLateralLoadCases2026);

% Optimize all member sections in the model, indicating how many 
%   non-linear iterations to perform before stopping, and what
%   factor of safety to use to design against local member failure
fprintf('optimizing member sections...\n')
Niterations = 5; % maximum number of iterations to run
safetyFactor = 1.4; % factor of safety to use while optimizing members
bridge.optimizeMemberSections(Niterations, safetyFactor);
bridge.computeMemberCapacities();

% Specify an output .dxf file (readable in AutoCAD), which will indicate
%   all of the optimized member cross-sections determined by the 
%   optimization procedure. These will be written to the file whose name
%   is indicated as the input to the "writeDXF" function
%   (e.g. 'optimized_example.dxf')
bridge.writeDXF('optimized_example.dxf')
