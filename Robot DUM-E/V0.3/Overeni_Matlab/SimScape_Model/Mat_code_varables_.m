clc 
close all
clear 

%% Parametry
d0 = 0.1207;
a1 = 0.265;
a2 = 0.22313448854;
a3 = 0.0995;
d4 = 0.1355;
d5 = 0.200;

%% parameters for model

g3 = 211.6244444;
T3 = [cosd(g3) -sind(g3) 0;
      sind(g3)  cosd(g3) 0;
      0         0        1];

g4 = 180;  
T4 = [1 0           0
      0 cosd(g4)    -sind(g4)
      0 sind(g4)    cosd(g4)];
  
  