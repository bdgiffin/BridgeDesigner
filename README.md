# BridgeDesigner
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.14261034.svg)](https://doi.org/10.5281/zenodo.14261034)

BridgeDesigner is a CAD-to-analysis and structural design optimization toolkit intended for use by teams participating in the [National Student Steel Bridge Competition (NSSBC)](https://www.aisc.org/education/university-programs/student-steel-bridge-competition).

BridgeDesigner comprises an integrated library of Matlab scripts which automate various structural modeling and design tasks:
- Automatically convert structural designs drafted in AutoCAD (saved in the `.dxf` drawing file format) into ready-to-use structural analysis models.
- Assign cross-sectional properties to structural members based upon the current catalog of all 4130 round and square tubing section sizes available from [Wicks Aircraft Parts and Supplies](https://www.wicksaircraft.com).
- Apply loads consistent with all defined vertical and lateral load cases specified in the NSSBC rules.
- Determine the deflected shape and internal member forces in the structure for all defined load cases using standard ["matrix" structural analysis methods](https://digitalcommons.bucknell.edu/books/7/).
- Measure and report the worst-case lateral sway among all defined lateral load cases.
- Measure and report vertical deflections at all deflection measurement locations (D1, D2) for all defined vertical load cases.
- Compute and report the statistically averaged structural efficiency score based upon the structure's estimated weight and aggregate deflection.
- Assess the worst-case demand-to-capacity ratios for all members considering various modes of failure (gross yielding, bending stress, torsional shear stress, and elastic buckling).
- Assess structural stability through a "linear buckling analysis" (refer to [Chapter 9 in Matrix Structural Analysis by McGuire et al.](https://digitalcommons.bucknell.edu/cgi/viewcontent.cgi?article=1006&context=books)), and report the worst-case elastic critical load factor.
- Visualize the defined model and analysis results, including the worst-case buckling mode shape, and the worst-case demand-to-capacity ratios for all structural members.
- Automatically resize the sections assigned to each member in the structure to minimize the overall efficiency score.
- Export optimized designs to AutoCAD (output in the `.dxf` drawing file format).

BridgeDesigner was originally written for the 2018 NSSBC rules, but has been updated for the [2025 NSSBC rules](https://www.aisc.org/globalassets/aisc/university-programs/ssbc/rules/ssbc-2025-rules.pdf).

Please direct all inquiries (including bug reports, feature requests, etc.) to Brian Doran Giffin ([brian.giffin@okstate.edu](mailto:brian.giffin@okstate.edu)), the original author of BridgeDesigner.

## Motivation

The NSSBC offers engineering students a valuable hands-on opportunity to learn about structural analysis and design. During the design phase of the project, student teams gain practical experience using a variety of software tools commonly used in a structural design setting (e.g. AutoCAD, SAP2000, Excel). The complete process of drafting designs (in AutoCAD), converting designs into structural analysis models (in SAP2000), and interpreting the analysis results to evaluate structural performance (in Excel) is a worthwhile experience that students are highly encouraged to learn about and fully participate in (the [NSSBC Design Tutorials](https://www.youtube.com/playlist?list=PLB_WnIf1Nsmq61g9-NIHmxg3-P95dDye9) explain this step-wise design process in greater detail).

The process of optimizing a given design to improve its overall performance is also a valuable exercise in learning about iterative design procedures. However, the model generation and data processing portions of this workflow tend to be the most time-consuming and tedious aspects of the overall design process. These tasks are not particularly educational in nature, but nonetheless consume the majority of one's time, detracting from opportunities to explore and compare a larger number of alternative designs. Previous efforts to automate these aspects of the design process (such as the [SBDesignTools](https://github.com/bdgiffin/SBDesignTools)) have been pursued and are publicly available, but such tools have become outdated and rely upon access to the commercial (i.e. non-student licence) version of SAP2000, which may not be available to all student project teams.

BridgeDesigner may be viewed as the culmination of previous CAD-to-analysis automation efforts, synthesizing the functionality of the SBDesignTools within an open-source structural analysis and optimization framework, requiring only that students have access to MATLAB. Providing students with access to these tools achieves a broad range of educational objectives:
 1. To minimize the amount of time students spend performing tedious model setup and data processing tasks, giving teams more time to explore innovative alternative design ideas.
 2. To create a tighter feedback loop between the drafting phase and the structural performance evaluation phase of the iterative design process, enhancing students' intuition and understanding of how various modifications to the design will result in better or worse structural performance.
 3. To maximize the number of student team members who wish to participate in the design phase of the project, requiring relatively limited prior knowledge of structural analysis or the NSSBC rules, and enabling consistent quantitative comparisons between candidate designs produced by different students.
 4. To provide students with a transparent window to see how common structural analysis software tools work "under the hood," and to give students the ability to modify or expand upon these capabilities while enhancing their computer programming literacy and their knowledge of structural analysis.
 5. To generate increased student interest and awareness of structural optimization and applied mathematics, demonstrating the utility of these fields of study in efficiently solving practical engineering design problems.
 6. To level the playing field between larger/established teams and smaller/burgeoning teams participating at different universities across the country.

## Quick Start Guide

To start using BridgeDesigner and exploring its different capabilities, the following step-by-step guide provides instructions for executing the example scripts (`example.m` and `optimization.m`) distributed with the source code:
1. First, you will need to download and install MATLAB on your computer (students at OSU can [download and license MATLAB for free](https://ceat.okstate.edu/itservices/software/mathworks-matlab-simulink.html)). If you are unfamilar with MATLAB, the MathWorks webpage has a number of useful tutorials to help you [get started with MATLAB](https://www.mathworks.com/help/matlab/getting-started-with-matlab.html).
2. Clone or [download a copy](https://github.com/bdgiffin/BridgeDesigner/archive/refs/heads/master.zip) of the BridgeDesigner source code repository from GitHub.
3. Open MATLAB and navigate to the BridgeDesigner folder on your computer.
4. Open the `example.m` script and read through the various comments explaining how the AutoCAD drawing file `example.dxf` is specified to be imported and converted into a structural analysis model within the BridgeBuilder framework.
   - If you wish to run this script using your own AutoCAD drawing file, you need only modify the `dxf`, `axes`, `active`, and `decking` variables within the `example.m` script (the rest of the script can be left as-is).
   - Make sure to save your AutoCAD drawing as a `.dxf` file, formatted similar to the provided `example.dxf` file (using the same unit system, with all measurements in inches.) 
5. Run the `example.m` script and observe the output printed to the MATLAB command window, including: the weight of the structure, the measured aggregate and lateral deflections, the estimated structural efficiency score, and the worst-case elastic buckling load factor.
   - Two interactive MATLAB figures will also be generated: the first figure displays the worst-case buckling mode shape, while the second figure displays the computed demand-to-capacity ratios for all structural members considering a variety of potential failure criteria (gross yielding, maximum bending/torsional stress, and elastic buckling).

The `optimization.m` script is formatted in a similar fashion to the `example.m` script, but exercises different functionality and produces different output:
1. Open the `optimization.m` script and read through the various comments explaining how the script works.
   - As with `example.m`, if you wish to run this script using your own AutoCAD drawing file, you will need to modify the `dxf`, `axes`, `active`, and `decking` variables accordingly.
2. Run the `optimization.m` script and observe the output printed to the MATLAB command window.
   - During the execution of this script, a fully automated iterative design optimization procedure is carried out to adjust the cross-sectional properties assigned to each member of the structure, considering the allowable list of alternative section sizes defined in the accompanying `defineProperties.m` function.
   - An updated estimate of the structural efficiency score is reported for each design iteration, ideally demonstrating convergence toward a fully optimized design.
   - After performing a user-specified number of design iterations (`Niterations = 5`, although the number of iterations can be changed to the user's preference), the iteration procedure is halted and the final set of optimized cross-sectional properties assigned to each member is displayed in a MATLAB figure.
   - Additionally, information regarding the specific section sizes assigned to all members is exported to an AutoCAD file named `optimized_example.dxf`.

For additional information regarding the functionality and organization of BridgeDesigner, refer to the [BridgeBuilder Manual](docs/BridgeBuilder_Manual.pdf).

Students are encouraged to expand upon the provided set of scripts and to write their own customized workflows to suit their needs.

> **CAUTION**: Future revisions to the NSSBC rules will warrant changes to several core BridgeBuilder functions. Scripted workflows from previous years' competitions may not be directly usable without appropriate modifications.
