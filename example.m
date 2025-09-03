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

% Compute the (average) efficiency score for the newly created structure
%   by invoking the "computeEfficiency" method defined on the
%   "Structure" class object.
fprintf('calculating score...\n')
[ scores, weight, deflections ] = bridge.computeEfficiency();
average_score = bridge.computeAverageEfficiency();
% (Optionally) print the results to the Matlab command window
fprintf('measured weight of structure = %6.2f lbs\n', weight)
fprintf('maximum aggregate deflection = %7.4f in\n', max(deflections))
import java.text.*; fmt = DecimalFormat; % (for printing comma-separated #'s)
fprintf('average efficiency score     = $%s\n', char(fmt.format(ceil(average_score))))

% Compute the worst-case lateral sway using the
%   "computeLateralSway" method
[ sway, swayID ] = bridge.computeLateralSway();
% (Optionally) print the result to the Matlab command window
fprintf('maximum lateral deflection   = %7.4f in (case %i)\n', sway, swayID)

% Compute the worst-case elastic buckling load factor
%   using the "computeBucklingLoad" method.
%   - The buckling load "factor" is the amount by which the applied loads
%     would need to be multiplied to induce the corresponding elastic
%     buckling "mode" of failure. That is to say, "factor" represents a
%     factor of safety in the design against elastic buckling, i.e.
%     - if factor > 1, then the structure is safe
%     - if factor < 1, then the structure will fail due to buckling
%   WARNING: Elastic buckling analyses are typically non-conservative
%            estimates of the actual buckling load for a structure.
%            As such, a fairly generous factor of safety is strongly
%            recommended (e.g. factor > 1.4)
fprintf('running buckling analysis...\n')
[ factor, mode, bucklingID ] = bridge.computeBucklingLoad();
% (Optionally) print the result to the Matlab command window
fprintf('elastic buckling load factor = %6.3f (case %i)\n', factor, bucklingID)
% (Optionally) plot the exaggerated deformed shape of the buckling mode
bridge.plotDeformed(20*mode);
fprintf('analysis complete!\n')

% Compute and plot the member capacities of all members in the structure
%   using the "computeMemberCapacities" method.
%   - All members are currently checked against combined: gross yield,
%     bending stress, torsional shear stress, and elastic buckling
fprintf('computing member capacities...\n')
bridge.computeMemberCapacities();
