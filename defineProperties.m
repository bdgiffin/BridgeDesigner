function [ steel, tubings ] = defineProperties()
% Define all material and tubing section properties
% (Updated based on Wicks Aircraft and Motorsports catalog on 9/7/2025)

% define the only material: 4130 steel
%                   rho      E      nu    Fy
%                (lb/in^3) (ksi) (in/in) (ksi)
steel = Material(  0.284,  29000,  0.3,  66.7 );

tubings = cell(141,1);
% define all circular tubing section sizes
%                            name          type    O.D.    t
%                                                  (in)   (in)
tubings{1}   = Section('R-0D1875X0D035' , 'PIPE' ,0.1875,0.035);
tubings{2}   = Section('R-0D2500X0D028' , 'PIPE' ,0.2500,0.028);
tubings{3}   = Section('R-0D2500X0D035' , 'PIPE' ,0.2500,0.035);
tubings{4}   = Section('R-0D2500X0D049' , 'PIPE' ,0.2500,0.049);
tubings{5}   = Section('R-0D2500X0D058' , 'PIPE' ,0.2500,0.058);
tubings{6}   = Section('R-0D2500X0D065' , 'PIPE' ,0.2500,0.065);
tubings{7}   = Section('R-0D3125X0D028' , 'PIPE' ,0.3125,0.028);
tubings{8}   = Section('R-0D3125X0D035' , 'PIPE' ,0.3125,0.035);
tubings{9}   = Section('R-0D3125X0D049' , 'PIPE' ,0.3125,0.049);
tubings{10}  = Section('R-0D3125X0D058' , 'PIPE' ,0.3125,0.058);
tubings{11}  = Section('R-0D3125X0D065' , 'PIPE' ,0.3125,0.065);
tubings{12}  = Section('R-0D3125X0D083' , 'PIPE' ,0.3125,0.083);
tubings{13}  = Section('R-0D3750X0D035' , 'PIPE' ,0.3750,0.035);
tubings{14}  = Section('R-0D3750X0D049' , 'PIPE' ,0.3750,0.049);
tubings{15}  = Section('R-0D3750X0D058' , 'PIPE' ,0.3750,0.058);
tubings{16}  = Section('R-0D3750X0D065' , 'PIPE' ,0.3750,0.065);
tubings{17}  = Section('R-0D3750X0D083' , 'PIPE' ,0.3750,0.083);
tubings{18}  = Section('R-0D3750X0D095' , 'PIPE' ,0.3750,0.095);
tubings{19}  = Section('R-0D4375X0D028' , 'PIPE' ,0.4375,0.028);
tubings{20}  = Section('R-0D4375X0D049' , 'PIPE' ,0.4375,0.049);
tubings{21}  = Section('R-0D4375X0D058' , 'PIPE' ,0.4375,0.058);
tubings{22}  = Section('R-0D4375X0D065' , 'PIPE' ,0.4375,0.065);
tubings{23}  = Section('R-0D4375X0D120' , 'PIPE' ,0.4375,0.120);
tubings{24}  = Section('R-0D5000X0D035' , 'PIPE' ,0.5000,0.035);
tubings{25}  = Section('R-0D5000X0D049' , 'PIPE' ,0.5000,0.049);
tubings{26}  = Section('R-0D5000X0D058' , 'PIPE' ,0.5000,0.058);
tubings{27}  = Section('R-0D5000X0D065' , 'PIPE' ,0.5000,0.065);
tubings{28}  = Section('R-0D5000X0D083' , 'PIPE' ,0.5000,0.083);
tubings{29}  = Section('R-0D5000X0D095' , 'PIPE' ,0.5000,0.095);
tubings{30}  = Section('R-0D5000X0D120' , 'PIPE' ,0.5000,0.120);
tubings{31}  = Section('R-0D5625X0D035' , 'PIPE' ,0.5625,0.035);
tubings{32}  = Section('R-0D5625X0D049' , 'PIPE' ,0.5625,0.049);
tubings{33}  = Section('R-0D5625X0D065' , 'PIPE' ,0.5625,0.065);
tubings{34}  = Section('R-0D5625X0D120' , 'PIPE' ,0.5625,0.120);
tubings{35}  = Section('R-0D6250X0D028' , 'PIPE' ,0.6250,0.028);
tubings{36}  = Section('R-0D6250X0D035' , 'PIPE' ,0.6250,0.035);
tubings{37}  = Section('R-0D6250X0D049' , 'PIPE' ,0.6250,0.049);
tubings{38}  = Section('R-0D6250X0D058' , 'PIPE' ,0.6250,0.058);
tubings{39}  = Section('R-0D6250X0D065' , 'PIPE' ,0.6250,0.065);
tubings{40}  = Section('R-0D6250X0D083' , 'PIPE' ,0.6250,0.083);
tubings{41}  = Section('R-0D6250X0D095' , 'PIPE' ,0.6250,0.095);
tubings{42}  = Section('R-0D6250X0D120' , 'PIPE' ,0.6250,0.120);
tubings{43}  = Section('R-0D6250X0D156' , 'PIPE' ,0.6250,0.156);
tubings{44}  = Section('R-0D7500X0D028' , 'PIPE' ,0.7500,0.028);
tubings{45}  = Section('R-0D7500X0D035' , 'PIPE' ,0.7500,0.035);
tubings{46}  = Section('R-0D7500X0D049' , 'PIPE' ,0.7500,0.049);
tubings{47}  = Section('R-0D7500X0D058' , 'PIPE' ,0.7500,0.058);
tubings{48}  = Section('R-0D7500X0D065' , 'PIPE' ,0.7500,0.065);
tubings{49}  = Section('R-0D7500X0D095' , 'PIPE' ,0.7500,0.095);
tubings{50}  = Section('R-0D7500X0D120' , 'PIPE' ,0.7500,0.120);
tubings{51}  = Section('R-0D7500X0D156' , 'PIPE' ,0.7500,0.156);
tubings{52}  = Section('R-0D7500X0D188' , 'PIPE' ,0.7500,0.188);
tubings{53}  = Section('R-0D8750X0D035' , 'PIPE' ,0.8750,0.035);
tubings{54}  = Section('R-0D8750X0D049' , 'PIPE' ,0.8750,0.049);
tubings{55}  = Section('R-0D8750X0D058' , 'PIPE' ,0.8750,0.058);
tubings{56}  = Section('R-0D8750X0D065' , 'PIPE' ,0.8750,0.065);
tubings{57}  = Section('R-0D8750X0D083' , 'PIPE' ,0.8750,0.083);
tubings{58}  = Section('R-0D8750X0D095' , 'PIPE' ,0.8750,0.095);
tubings{59}  = Section('R-0D8750X0D120' , 'PIPE' ,0.8750,0.120);
tubings{60}  = Section('R-0D8750X0D188' , 'PIPE' ,0.8750,0.188);
tubings{61}  = Section('R-1D0000X0D028' , 'PIPE' ,1.0000,0.028);
tubings{62}  = Section('R-1D0000X0D035' , 'PIPE' ,1.0000,0.035);
tubings{63}  = Section('R-1D0000X0D049' , 'PIPE' ,1.0000,0.049);
tubings{64}  = Section('R-1D0000X0D058' , 'PIPE' ,1.0000,0.058);
tubings{65}  = Section('R-1D0000X0D065' , 'PIPE' ,1.0000,0.065);
tubings{66}  = Section('R-1D0000X0D083' , 'PIPE' ,1.0000,0.083);
tubings{67}  = Section('R-1D0000X0D095' , 'PIPE' ,1.0000,0.095);
tubings{68}  = Section('R-1D0000X0D120' , 'PIPE' ,1.0000,0.120);
tubings{69}  = Section('R-1D0000X0D156' , 'PIPE' ,1.0000,0.156);
tubings{70}  = Section('R-1D0000X0D188' , 'PIPE' ,1.0000,0.188);
tubings{71}  = Section('R-1D0000X0D250' , 'PIPE' ,1.0000,0.250);
tubings{72}  = Section('R-1D1250X0D035' , 'PIPE' ,1.1250,0.035);
tubings{73}  = Section('R-1D1250X0D049' , 'PIPE' ,1.1250,0.049);
tubings{74}  = Section('R-1D1250X0D058' , 'PIPE' ,1.1250,0.058);
tubings{75}  = Section('R-1D1250X0D065' , 'PIPE' ,1.1250,0.065);
tubings{76}  = Section('R-1D1250X0D095' , 'PIPE' ,1.1250,0.095);
tubings{77}  = Section('R-1D1250X0D120' , 'PIPE' ,1.1250,0.120);
tubings{78}  = Section('R-1D1250X0D250' , 'PIPE' ,1.1250,0.250);
tubings{79}  = Section('R-1D2500X0D035' , 'PIPE' ,1.2500,0.035);
tubings{80}  = Section('R-1D2500X0D049' , 'PIPE' ,1.2500,0.049);
tubings{81}  = Section('R-1D2500X0D058' , 'PIPE' ,1.2500,0.058);
tubings{82}  = Section('R-1D2500X0D065' , 'PIPE' ,1.2500,0.065);
tubings{83}  = Section('R-1D2500X0D083' , 'PIPE' ,1.2500,0.083);
tubings{84}  = Section('R-1D2500X0D095' , 'PIPE' ,1.2500,0.095);
tubings{85}  = Section('R-1D2500X0D120' , 'PIPE' ,1.2500,0.120);
tubings{86}  = Section('R-1D2500X0D156' , 'PIPE' ,1.2500,0.156);
tubings{87}  = Section('R-1D2500X0D250' , 'PIPE' ,1.2500,0.250);
tubings{88}  = Section('R-1D3750X0D035' , 'PIPE' ,1.3750,0.035);
tubings{89}  = Section('R-1D3750X0D049' , 'PIPE' ,1.3750,0.049);
tubings{90}  = Section('R-1D3750X0D058' , 'PIPE' ,1.3750,0.058);
tubings{91}  = Section('R-1D3750X0D065' , 'PIPE' ,1.3750,0.065);
tubings{92}  = Section('R-1D3750X0D095' , 'PIPE' ,1.3750,0.095);
tubings{93}  = Section('R-1D3750X0D120' , 'PIPE' ,1.3750,0.120);
tubings{94}  = Section('R-1D3750X0D250' , 'PIPE' ,1.3750,0.250);
tubings{95}  = Section('R-1D5000X0D035' , 'PIPE' ,1.5000,0.035);
tubings{96}  = Section('R-1D5000X0D049' , 'PIPE' ,1.5000,0.049);
tubings{97}  = Section('R-1D5000X0D058' , 'PIPE' ,1.5000,0.058);
tubings{98}  = Section('R-1D5000X0D065' , 'PIPE' ,1.5000,0.065);
tubings{99}  = Section('R-1D5000X0D083' , 'PIPE' ,1.5000,0.083);
tubings{100} = Section('R-1D5000X0D095' , 'PIPE' ,1.5000,0.095);
tubings{101} = Section('R-1D5000X0D120' , 'PIPE' ,1.5000,0.120);
tubings{102} = Section('R-1D5000X0D188' , 'PIPE' ,1.5000,0.188);
tubings{103} = Section('R-1D6250X0D049' , 'PIPE' ,1.6250,0.049);
tubings{104} = Section('R-1D6250X0D065' , 'PIPE' ,1.6250,0.065);
tubings{105} = Section('R-1D6250X0D083' , 'PIPE' ,1.6250,0.083);
tubings{106} = Section('R-1D6250X0D095' , 'PIPE' ,1.6250,0.095);
tubings{107} = Section('R-1D6250X0D120' , 'PIPE' ,1.6250,0.120);
tubings{108} = Section('R-1D6250X0D156' , 'PIPE' ,1.6250,0.156);
tubings{109} = Section('R-1D6250X0D188' , 'PIPE' ,1.6250,0.188);
tubings{110} = Section('R-1D7500X0D049' , 'PIPE' ,1.7500,0.049);
tubings{111} = Section('R-1D7500X0D058' , 'PIPE' ,1.7500,0.058);
tubings{112} = Section('R-1D7500X0D065' , 'PIPE' ,1.7500,0.065);
tubings{113} = Section('R-1D7500X0D095' , 'PIPE' ,1.7500,0.095);
tubings{114} = Section('R-1D7500X0D120' , 'PIPE' ,1.7500,0.120);
tubings{115} = Section('R-1D7500X0D188' , 'PIPE' ,1.7500,0.188);
tubings{116} = Section('R-2D0000X0D049' , 'PIPE' ,2.0000,0.049);
tubings{117} = Section('R-2D0000X0D065' , 'PIPE' ,2.0000,0.065);
tubings{118} = Section('R-2D0000X0D095' , 'PIPE' ,2.0000,0.095);
tubings{119} = Section('R-2D0000X0D120' , 'PIPE' ,2.0000,0.120);
tubings{120} = Section('R-2D0000X0D188' , 'PIPE' ,2.0000,0.188);
tubings{121} = Section('R-2D2500X0D120' , 'PIPE' ,2.2500,0.120);
tubings{122} = Section('R-2D2500X0D250' , 'PIPE' ,2.2500,0.250);
tubings{123} = Section('R-2D5000X0D120' , 'PIPE' ,2.5000,0.120);
tubings{124} = Section('R-2D5000X0D250' , 'PIPE' ,2.5000,0.250);

% define all square tubing section sizes
%                            name          type      w     t
%                                                  (in)  (in)
tubings{125} = Section('S-0D3750X0D035' , 'TUBE' ,0.3750,0.035);
tubings{126} = Section('S-0D5000X0D035' , 'TUBE' ,0.5000,0.035);
tubings{127} = Section('S-0D5000X0D095' , 'TUBE' ,0.5000,0.095);
tubings{128} = Section('S-0D6250X0D049' , 'TUBE' ,0.6250,0.049);
tubings{129} = Section('S-0D6250X0D065' , 'TUBE' ,0.6250,0.065);
tubings{130} = Section('S-0D7500X0D035' , 'TUBE' ,0.7500,0.035);
tubings{131} = Section('S-0D7500X0D049' , 'TUBE' ,0.7500,0.049);
tubings{132} = Section('S-0D7500X0D065' , 'TUBE' ,0.7500,0.065);
tubings{133} = Section('S-0D8750X0D035' , 'TUBE' ,0.8750,0.035);
tubings{134} = Section('S-0D8750X0D049' , 'TUBE' ,0.8750,0.049);
tubings{135} = Section('S-0D8750X0D065' , 'TUBE' ,0.8750,0.065);
tubings{136} = Section('S-1D0000X0D035' , 'TUBE' ,1.0000,0.035);
tubings{137} = Section('S-1D0000X0D049' , 'TUBE' ,1.0000,0.049);
tubings{138} = Section('S-1D0000X0D058' , 'TUBE' ,1.0000,0.058);
tubings{139} = Section('S-1D0000X0D065' , 'TUBE' ,1.0000,0.065);
tubings{140} = Section('S-1D1250X0D049' , 'TUBE' ,1.1250,0.049);
tubings{141} = Section('S-1D2500X0D058' , 'TUBE' ,1.2500,0.058);

end
