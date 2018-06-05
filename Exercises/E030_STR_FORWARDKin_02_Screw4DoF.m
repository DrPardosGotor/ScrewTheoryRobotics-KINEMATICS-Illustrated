%% Screw Theory for Robotics - FORWARD Kinematics - Exercise_02.
% Screw Theory - Forward kinematics - 4 DoF Manipulator Exercise
% Calculate the Homogeneous Matrix transformation for the end-effector of
% robot with four links (Rot1 + Tra2+ Tra3 + Rot4) whose "Twist-Mangitude"
% parameters are:
% AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
% or =[0 0 0]'; l1 = [0 0 1]'; l4 = [4 0 0]';
% Axis = [AxisZ AxisZ AxisX AxisX];
% Point = [or or l1 l1];
% Joint = ['rot'; 'tra'; 'tra'; 'rot'];
%
% Mag = [pi 2 3 pi/4];
% The result is:
% Hst = [-S1C4  S1S4 C1 C1(Mag3+l4);
%         C1C4 -C1S4 S1 S1(Mag3+l4);
%           S4   C4   0     Mag2+l1;
%            0    0   0         1].
% ans =
%   -0.0000    0.0000   -1.0000   -7.0000
%   -0.7071    0.7071    0.0000    0.0000
%    0.7071    0.7071         0    3.0000
%         0         0         0    1.0000
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
%% E030_STR_FORWARDKin_02_Screw4DoF
clear;
clc;
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
or =[0 0 0]'; l1 = [0 0 1]'; l4 = [4 0 0]';
Axis = [AxisZ AxisZ AxisX AxisX];
Point = [or or l1 l1];
Joint = ['rot'; 'tra'; 'tra'; 'rot'];
Twist = Joint2Twist(Axis(:,1), Point(:,1), Joint(1,:));
for i = 2:size(Point,2)
    Twist = [Twist Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:))];
end
Mag = [pi 2 3 pi/4];
TwMag = [Twist; Mag];
HstR = ForwardKinematicsPOE(TwMag);
Hst0 = trvZ2tform(l1(3))*trvX2tform(l4(1))*rotX2tform(pi/2)*rotY2tform(pi/2);
Hst1 = HstR * Hst0
% "ForwardKinematicsPOE" by "DrJMPG Robotics Screw Theory Toolbox" Dr. Pardos-Gotor.
% To check that the "ForwardKinematicsDH" works properly for Hst1, we 
% compute the direct result Hst for Hst2.
S1 = sin(TwMag(7,1));
C1 = cos(TwMag(7,1));
S4 = sin(TwMag(7,4));
C4 = cos(TwMag(7,4));
Mag2 = TwMag(7,2);
Mag3 = TwMag(7,3);
Hst2 = [-S1*C4  S1*S4 C1 C1*(Mag3+l4(1)); C1*C4 -C1*S4 S1 S1*(Mag3+l4(1)); S4 C4 0 Mag2+l1(3); 0 0 0 1]
%