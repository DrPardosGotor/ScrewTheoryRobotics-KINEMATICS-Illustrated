%% Screw Theory for Robotics - FORWARD Kinematics - Exercise_01.
% Denavit-Hartenberg - Forward kinematics - 4 DoF Manipulator Exercise  
% Calculate the Homogeneous Matrix transformation for the end-effector of
% robot with four links (Rot1 + Tra2+ Tra3 + Rot4) whose DH parameters are:
% dhparams = [1 pi 0 0; 2 pi/2 0 pi/2; 3 0 0 0; 4 pi/4 0 0];
% The result is:
% Hst = [-S1C4  S1S4 C1 C1(d3+d4);
%         C1C4 -C1S4 S1 S1(d3+d4);
%           S4   C4   0     d2+d1;
%            0    0   0         1].
% To check that the "ForwardKinematicsDH" works properly for Hst1, we 
% compute the direct result Hst for Hst2.
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
%% E030_STR_FORWARDKin_01_DH4DoF
clear;
clc;
dhparams = [1 pi 0 0; 2 pi/2 0 pi/2; 3 0 0 0; 4 pi/4 0 0];
Hst1 = ForwardKinematicsDH(dhparams)
S1 = sin(dhparams(1,2));
C1 = cos(dhparams(1,2));
S4 = sin(dhparams(4,2));
C4 = cos(dhparams(4,2));
d1 = dhparams(1,1);
d2 = dhparams(2,1);
d3 = dhparams(3,1);
d4 = dhparams(4,1);
Hst2 = [-S1*C4  S1*S4 C1 C1*(d3+d4); C1*C4 -C1*S4 S1 S1*(d3+d4); S4 C4 0 d2+d1; 0 0 0 1]
%