%% Screw Theory - CANONICAL Inverse Kinematics - Exercise_03
% Paden-Kahan THREE (PK3).
%
% Calculate IK for a single movement using PadenKahanPardosThree function.
% computing the Theta of the Screw with twist x1, to move the point pp
% from its original position to the point "c" or "e", complying that the
% distance between "c" or "e" to a point pk is "de".
% the movement is defined by the SCREW whose "Twist" parameters
% are defined by: Axis = Axis1, Point = p1, JointType = 'rot' or 'tra';
% and whose magnitude is defined by Mag.
%
% The movement is aplied to the point pp for moving it to a certain point
% (c or e) whose distance to pk is given for the distance de.
%
% For checking the working of the PKP3 this exercise has three steps:
% STEP1: Apply ForwardKinemats to the Screw for "whatever" Mag (can be
% even more than 2pi), on pp and then getting a feasible "c" for 
% calculating the distance "de" between "c" and the goal "pk".
% STEP2: Calculate the IK solution by PKP3 getting the magnitud Theta1
% STEP3: Test the PKP3 solution applying ForwardKinemats to the Screw with
% Theta1 = [t11;t12] on pp and checking we get the same distance "de" to pk
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
% This file is part of The ST24R "Screw Theory Toolbox for Robotics" MATLAB
% 
% ST24R is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published
% by the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ST24R is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Lesser General Public License
% along with ST24R.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E050_STR_CANONICALInvKin_03_PadenKahanTHREE
%
clear
clc
%
pp = [rand*10 rand*10 rand*10]' % for testing various initial points
pk = [rand*10 rand*10 rand*10]' % for testing various final points
Mag = (rand-rand)*2*pi; % for testing various magnitudes
%
Axis1 = [1 0 0]';
p1 = [0 0 0]';
JointType1 = 'tra';
Twist = Joint2Twist(Axis1, p1, JointType1);
%
% STEP1: Apply ForwardKinemats to the Screw for "whatever" Mag
TwMag1 = [Twist; Mag];
HstR1 = ForwardKinematicsPOE(TwMag1);
pc1h = HstR1*[pp; 1];
pc1 = pc1h(1:3);
de1 = norm(pk-pc1)
%
% STEP2: Calculate the IK solution by PKP3 getting the magnitud Theta1
Theta1 = PadenKahanPardosThree(Twist, pp, pk, de1)
%
% STEP3: Test the PK1 solution applying ForwardKinemats to the Screw
TwMag2 = [Twist; Theta1(1)];
HstR2 = ForwardKinematicsPOE(TwMag2);
pc2h = HstR2*[pp; 1];
pc2 = pc2h(1:3);
de2 = norm(pk-pc2)
TwMag3 = [Twist; Theta1(2)];
HstR3 = ForwardKinematicsPOE(TwMag3);
pe3h = HstR3*[pp; 1];
pe3 = pe3h(1:3);
de3 = norm(pk-pe3)
% Check that (de1 = de2 = de3) 
%