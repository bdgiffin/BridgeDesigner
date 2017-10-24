% ======================================================================= %
%             EXAMPLE BRIDGE-DESIGNER OPTIMIZATION SCRIPT                 %
% ======================================================================= %
%   BridgeDesigner is a Matlab library of structural design tools,        %
%   which can be used to write scripts for the purposes of automating     %
%   various design tasks. BridgeDesigner was originally written for the   %
%   2018 NSSBC rules, but may be extended for use in other settings.      %
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
%            layer        section
active = { 'Decking',  'SQ1X035-41';
           'Webbing',  'R3/16X035-41';
           'Laterals', 'R1/2X028-41';
           'Chords',   'R1-1/4X035-41';
           'Tendons',  'R1/2X028-41';
           'Legs',     'R1X028-41' };

% Create the analysis model as a "Structure" class object, indicating
%   the "dxf" file, coordinate "axes", and "active" layers,
%   defined previously.
fprintf('creating model...\n')
bridge = Structure(dxf, axes, active);

% Optimize all member sections in the model, indicating how many 
%   non-linear iterations to perform before stopping, and what
%   factor of safety to use to design against local member failure
fprintf('optimizing member sections...\n')
Niterations = 3; % maximum number of iterations
safetyFactor = 1.2; % factor of safety
bridge.optimizeMemberSectionsExplicit(Niterations, safetyFactor);
bridge.plotUndeformed()