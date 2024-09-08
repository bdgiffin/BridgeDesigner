# BridgeDesigner

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

The process of optimizing a given design to improve its overall performance is also a valuable exercise in learning about iterative design procedures. However, the model generation and data processing portions of this workflow tend to be the most time-consuming and tedious aspects of the overall design process. These tasks are not particularly educational in nature, but nonetheless consume the majority of one's time, detracting from opportunities to explore and compare a larger number of alternative designs. Previous efforts to automate these aspects of the design process (such as the [SBDesignTools](https://github.com/bdgiffin/SBDesignTools)) have been pursued and are publicly available.

While the rules and scoring criteria for the competition are intended to emulate design 

## Quick Start Guide

