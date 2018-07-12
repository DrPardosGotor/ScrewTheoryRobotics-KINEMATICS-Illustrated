%% Screw Theory - EXAMPLES Inverse Kinematics - Exercise_60.
% GANTRY robot - ABB IRB IRB6620LX.
%
% The goal of this exercise is to TEST:
% INVERSE KINEMATICS
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Mechanical characteristics of the Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pu = point in the axis of Th1(tra).
% pk = point in the axis of Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) rot+tra at robot reference (home configuration).
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% so getting a feasible TcP configuration (rot+tra) as Hst.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...6. There can be up to 4 right solutions
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% with Theta = [t11...t61; t12...t62; ...; t18...t68] and checking we get
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
%% E060_STR_EXAMPLESInvKin_60_ABBIRB6620LX
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
po=[0;0;0]; pu=[1088;2500;0]; % In fact pu has not use because Theta1=TRA
pk=[1.468;2.500;0]; pr=[2.443;2.500;0];
pf=[2.643;1.613;0]; pp=[3.000;1.613;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pu pk pr pf pf pp];
Joint = ['tra'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisZ AxisZ AxisZ -AxisY AxisZ AxisX];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotY2tform(pi/2);
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%
Theta = zeros(4,6);
%
% STEP2: Calculate the IK solutions Theta using the SCREW THEORY techniques
% and basically the PADEN-KAHAN-PARDOS Canonic Subproblems.
tic
% Calculate the IK solutions Theta using the SCREW THEORY techniques
% and basically the PADEN-KAHAN-PARDOS Canonic Subproblems.
%
% STEP2-1: Calculate Theta1.
% With "pf" on the axis of E4, E5, E6. We apply (noap*hs0^-1) to "pf"
% Doing so we get Theta1 applying the Canonic problem PARDOS-ONE,
% because the screws E4,E5,E6 do not affect "pf" for being on their axes
% and the E2,E3 do not change the plane where "pf" moves (perpendicular to
% the axis of those screws, and so do not affect the calculation for Theta1
% resulting the problem "exp(E1^theta1)*pf = noap*hs0^-1*pf" by PARDOS-ONE
% which has one solution for t11.
noapHst0if = noap*(Hst0\[pf; 1]); pk1 = noapHst0if(1:3);
Theta(1,1) = PadenKahanPardosOne(Twist(:,1), pf, pk1);
% prepare Theta for next calculation
Theta(2,:) = Theta(1,:);
%
% STEP2-2: Calculate Theta2 & Theta3.
% With "pf" on the axis of E4, E5, E6 we apply (noap*hs0^-1) to "pf" and
% the POE E1..E6 also to "pf" having already known the value for Theta1
% resulting exactly a Canonic problem PARDOS-FOUR, because the screws
% E4,E5,E6 do not affect "pf" and the E1 is known,resulting the problem
% exp(E2^theta2)*exp(E3^theta3)*pf = exp(E1^Th1)^-1*noap*gs0^-1*pf = pk1p
% which by PARDOS-FOUR has none, one or two DOUBLE solutions.
% t21-t31 & t22-t32 for each value of t11
%
E1inoapHst0if = (expScrew([Twist(:,1);Theta(1,1)]))\noapHst0if;
pk2 = E1inoapHst0if(1:3);
Theta(1:2,2:3) = PardosGotorFour(Twist(:,2),Twist(:,3),pf,pk2);
% prepare Theta for next calculation
Theta(3:4,:) = Theta(1:2,:); 
%
% STEP2-3: Calculate Theta4 & Theta5.
% With "pp" on the axis of E6 apply E3^-1*E2^-1*E1^-1*noap*gs0^-1 to "pp"
% and also the POE E4*E5*E6 to "pp" knowing already Theta3-Theta2-Theta1,
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E6 does not affect "pp" & Th3-Th2-Th1 known (four solutions), the problem
% exp(E4^theta4)*exp(E5^theta5)*pp = pk2p ; with
% pk2p = exp(E3^Th3)^-1*exp(E2^Th2)^-1*exp(E1^Th1)^-1*noap*gs0^-1*pp 
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions:
% t31,t21,t11 to t41-t51 & t42-t52 ; t32,t22,t11 to t43-t53 & t44-t54
%
noapHst0ip = noap*(Hst0\[pp; 1]); 
for i = 1:2
    pk3 = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0ip;
    pk3 = (expScrew([Twist(:,2);Theta(i,2)]))\pk3;
    pk3 = (expScrew([Twist(:,3);Theta(i,3)]))\pk3;
    pk3 = pk3(1:3);
    t4t5 = PadenKahanPardosTwo(Twist(:,4),Twist(:,5),pp,pk3);
    Theta(i,4:5) = t4t5(1,:); Theta(i+2,4:5) = t4t5(2,:);
end
%
% STEP4: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 solutions,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (four solutions) we get t6:
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:4
    pk4 = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
    pk4 = (expScrew([Twist(:,2);Theta(i,2)]))\pk4;
    pk4 = (expScrew([Twist(:,3);Theta(i,3)]))\pk4;
    pk4 = (expScrew([Twist(:,4);Theta(i,4)]))\pk4;
    pk4 = (expScrew([Twist(:,5);Theta(i,5)]))\pk4;
    pk4 = pk4(1:3);
    Theta(i,6) = PadenKahanPardosOne(Twist(:,6), po, pk4);
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