%% Screw Theory - EXAMPLES Inverse Kinematics - Exercise_70.
% SCARA robot - ABB IRB 910SC.
%
% Solves rhe INVERSE KINEMATICS for any desired position & orientation
% of the TCP (noap goal) of the ABB IRB910SC Robot.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
% Approach by Pardos1+Pardos4+PadenKahan1
%
% Mechanical characteristics of the Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pr = point in the axis of Th2(rot).
% pf = point in the axis of Th3(tra) and Th4(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) rot+tra at robot reference (home configuration).
%
% hst0 = Tool (TcP) configuration (rot+tra) at robot reference position. 
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% so getting a feasible TcP configuration (rot+tra) as Hst.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...6. There can be up to 8 right solutions for this
% problem using this approach (theoretically there is max of 16 solutions).
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% with Theta = [t11...t41; t12...t42] and checking we get
% the same TcP configuration (rot+tra) as Hst.
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
%% E060_STR_EXAMPLESInvKin_70_ABBIRB910SC
%
clear
clc
%
Mag = zeros(1,6);
for i = 1:6
    Mag(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
%
% Mechanical characteristics of the Robot:
po=[0;0;0]; pr=[0.4;0;0]; pf=[0.65;0;0]; pp=[0.65;0.125;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pr pf pp];
Joint = ['rot'; 'rot'; 'tra'; 'rot'];
Axis = [AxisY AxisY -AxisY -AxisY];
Twist = zeros(6,6);
for i = 1:4
    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotX2tform(pi/2);
%
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%
%
% STEP2: Calculate the IK solutions Theta using the SCREW THEORY techniques
Theta = zeros(2,6);
tic % start the ticking for calcule the performance of this algorithm.
%
% STEP2: Calculate Theta3.
% With "pf" on the axis of E4. We apply (noap*hs0^-1) to "pf"
% doing so we get Theta3 applying the Canonic problem PARDOS-ONE,
% because the screws E4 do not affect "pf" for being on its axis
% and the E1,E2 do not change the plane where "pf" moves (perpendicular to
% the axis of those screws, and so do not affect the calculation for Theta3
% resulting the problem "exp(E3^theta3)*pf = noap*hst0^-1*pf" by PARDOS-ONE
% which has one solution for t31.
pkp = noap*(Hst0\[pf; 1]);
Theta(1,3) = PardosGotorOne(Twist(:,3), pf, pkp(1:3));
% prepare Theta for next calculation
Theta(2,3) = Theta(1,3);
%
% STEP2: Calculate Theta1 & Theta2.
% With "pf" on the axis of E4, we apply (noap*hs0^-1) to "pf" and
% the POE E1..E4 also to "pf" having already known the value for Theta3
% resulting exactly a Canonic problem PARDOS-FOUR, because the screw
% E4 do not affect "pf" and the E3 is known,resulting the problem
% exp(E1^theta1)*exp(E2^theta2)*exp(E3^theta3)*pf = 
% exp(E1^theta1)*exp(E2^theta2)*pfp = noap*hst0^-1*pf = pkp
% which by PARDOS-FOUR has none, one or two DOUBLE solutions.
% t11-t21 & t12-t22 for each value of t31
%
pfp = expScrew([Twist(:,3);Theta(1,3)])*[pf; 1];
Theta(1:2,1:2) = PardosGotorFour(Twist(:,1),Twist(:,2),pfp(1:3),pkp(1:3));
%
% STEP2: Calculate Theta4.
% With "po" not in the axis of E4 apply E3^-1...*E1^-1*noap*hst0^-1 to "po"
% and applying E4 to "po" knowing already Theta3...Theta1 solutions,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E4^theta4)*po = pk3p ; with
% pk3p = exp(E3^Th3)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th3-Th2-Th1 known (two solutions) we get t4:
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:2
    pk3p = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
    pk3p = (expScrew([Twist(:,2);Theta(i,2)]))\pk3p;
    pk3p = (expScrew([Twist(:,3);Theta(i,3)]))\pk3p;
    Theta(i,4) = PadenKahanOne(Twist(:,4), po, pk3p(1:3));
end
%
Theta
tIKSTR = round(toc*1000,1);
time_IK = ['Time to solve IK Screw Theory ', num2str(tIKSTR),' ms']
% STEP3: Test the different solutions applying ForwardKinemats to Robot
for i = 1:size(Theta,1)
    TwMagi = [Twist; Theta(i,:)];
    HstRi = ForwardKinematicsPOE(TwMagi);
    i
    noapi = HstRi * Hst0
end
%
% Check that TcP POSE (rot+tra) as Hst is OK for all Theta values
%