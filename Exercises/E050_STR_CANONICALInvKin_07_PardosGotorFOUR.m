%% Screw Theory - CANONICAL Inverse Kinematics - Exercise_07
% Pardos-Gotor FOUR (PG4).
%
% Calculate IK for two consecutive parallel rotation SCREWS by PsrdosFOUR.
% the movements are defined by the SCREWS whose "Twists" parameters
% are: Axis = [Axis1 Axis2], Point = [p1 p2], JointType = 'rot'
% and whose magnitudes are defined by Mag = [Theta1 Theta2].
%
% The magnitude Theta2 is aplied to point pp for moving it to pc (or pd)
% then the magnitude Theta1 is aplied to pc (or pd) for moving them to pk.
%
% For checking the PardosFOUR function, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats to the Screws for "whatever" Mag (t2 + t1)
% (can be even more than 2pi) on pp and then getting a feasible pk.
% STEP2: Calculate the IK solution by PKP2 getting the magnitud
% Theta1Theta2 = [t11 t21; t12 t22] DOUBLE SOLUTION.
% STEP3: Test the TWO DOUBLE solutions got by PKP2 Theta1 & Theta2 applying
% ForwardKinemats to the Screws on pp and checking we get the same pk.
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
% You should have received a copy of the GNU Leser General Public License
% along with ST24R. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E050_STR_CANONICALInvKin_06_PardosGotorFOUR
%
clear
clc
%
pp = [rand*10 rand*10 rand*10]' % for testing various initial points
Mag = [(rand-rand)*2*pi (rand-rand)*2*pi]; % for testing various magnitudes
%
p1 = [0 0 0]'; p2 = [0 1 0]'; % points for the Screw Axes.
Point = [p1 p2];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
Axis = [AxisX AxisX]; % whatever for testing the exercise
JointType = ['rot'; 'rot']; % whatever for testing the exercise
% 
% Now we build the TWISTS matrix for the chosen Joints
Twist = Joint2Twist(Axis(:,1), Point(:,1), JointType(1,:));
for i = 2:size(Point,2)
    Twist = [Twist Joint2Twist(Axis(:,i), Point(:,i), JointType(i,:))];
end
%
% STEP1: Apply ForwardKinemats to the TWO Screws x2 and then x1 on pp for
% "whatever" Mag (can be% even more than 2pi) for getting a feasible pk.
TwMag1 = [Twist; Mag];
HstR1 = ForwardKinematicsPOE(TwMag1);
pk1h = HstR1*[pp; 1];
pk1 = pk1h(1:3)
% STEP2: Calculate the IK solution by PK2 getting the magnitud
% Theta1Theta2 = [t11 t21; t12 t22] DOUBLE SOLUTION.
Th1Th2 = PardosGotorFour(Twist(:,1), Twist(:,2), pp, pk1)
%
% STEP3: Test the TWO DOUBLE solutions by PK2 Theta1 & Theta2 applying
% ForwardKinemats to the Screws on pp and checking we get the same pk.
TwMag2 = [Twist; Th1Th2(1,:)];
HstR2 = ForwardKinematicsPOE(TwMag2);
pk2h = HstR2*[pp; 1];
pk2 = pk2h(1:3)
TwMag3 = [Twist; Th1Th2(2,:)];
HstR3 = ForwardKinematicsPOE(TwMag3);
pk3h = HstR3*[pp; 1];
pk3 = pk3h(1:3)
%
% Check that (pk1 = pk2 = pk3) 
%