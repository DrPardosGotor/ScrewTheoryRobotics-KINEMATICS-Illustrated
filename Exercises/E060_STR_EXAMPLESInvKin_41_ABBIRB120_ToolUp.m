%% Screw Theory - EXAMPLES Inverse Kinematics - Exercise_41.
%v ABB IRB120 Robot ToolUp.
%
% The goal of this exercise is to TEST:
% INVERSE KINEMATICS for IRB120 ToolUp POSE
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Mechanical characteristics of the Robot (AT REF POSITION):
% po = Origen for he STATIONARY system of reference.
% pk = point in the crossing of the DOF Th1(rot) & Th2(rot).
% pr = point in the axis of Th3(rot).
% pf = point in the crossing of the DOF Th4(rot), Th5(rot), Th6(rot).
% pp = TcP Tool Center Point
% hst0 = Tool (TcP) configuration (rot+tra) at robot reference position. 
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% so getting a feasible TcP configuration (rot+tra) as Hst.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...6. There can be up to 8 right solutions for this
% problem using this approach (theoretically there is max of 16 solutions).
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
%% E060_STR_EXAMPLESInvKin_41_ABBIRB120_ToolUp
%
clear
clc
%
Mag = zeros(1,6);
for i = 1:6
    Mag(i) = (rand-rand)*pi; % for testing various Theta1-Theta6
end
%
% Mechanical characteristics of the IRB120 Robot:
po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
pf=[0.302;0.630;0]; pp=[0.302;0.790;0];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pk pr pf pf pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisY AxisZ AxisZ AxisX AxisZ AxisY];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotX2tform(-pi/2)*rotZ2tform(pi/2);
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; Mag]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%
% STEP2: Calculate the IK solutions Theta using the SCREW THEORY techniques
% and basically the PADEN-KAHAN-PARDOS Canonic Subproblems.
Theta = zeros(8,6);
tic % start the ticking for calcule the performance of this algorithm.
%
% STEP2-1: Calculate Theta3.
% With "pf" on the axis of E4, E5, E6 and "pk" on the axis of E1, E2.
% We apply (noap*gs0^-1) to "pf" and take the norm of the diffence of that
% resulting point and "pk". Doing so we can calculate Theta3 applying the
% Canonic problem PADEN-KAHAN-THREE, because the screws E4,E5,E6 do not affect
% "pf" and the E1,E2 do not affect the norm of a vector with an end on "pk"
% resulting the problem ||exp(E3^theta3)*pf-pk||=||noap*gs0^-1*pf-pk||
% which by PADEN-KAHAN-THREE has none, one or two solutions for t31 t32.
noapHst0if = noap*(Hst0\[pf; 1]); pkp = noapHst0if(1:3);
de = norm(pkp - pk);
t3 = PadenKahanPardosThree(Twist(:,3), pf, pk, de);
Theta(1,3) = t3(1); Theta(5,3) = t3(2); % put results into Theta
Theta(3,:) = Theta(1,:); Theta(7,:) = Theta(5,:); % prepare Theta for next
%
% STEP2-2: Calculate Theta1 & Theta2.
% With "pf" on the axis of E4, E5, E6 we apply (noap*gs0^-1) to "pf" and
% the POE E1..E6 also to "pf" having already known the value for Theta3
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E4,E5,E6 do not affect "pf" and the E3 is known (two values),resulting
% the problem exp(E1^theta1)*exp(E2^theta2)*pf' = noap*gs0^-1*pf
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions
% t11-t21 & t12-t22 for each value of t3, but we have two, then consider
% for t31 we get t11-t21 & t12-t22 & for t32 we get t13-t23 & t14-t24.
t1t2 = zeros(4,2);
for i = 1:2                   % for the TWO values of t3.
    j = i+fix(i/2);
    pfpt = expScrew([Twist(:,3);t3(i)])*[pf; 1];
    pfp = pfpt(1:3);
    t1t2(j:j+1,1:2) = PadenKahanPardosTwo(Twist(:,1),Twist(:,2),pfp,pkp);
end
for i = 1:2:7
    j = i-fix(i/2);
    Theta(i,1:2) = t1t2(j,1:2); % put t1t2 values into Theta
    Theta(i+1,:) = Theta(i,:);  % prepare Theta for next step
end
%
% STEP2-3: Calculate Theta4 & Theta5.
% With "pp" on the axis of E6 apply E3^-1*E2^-1*E1^-1*noap*gs0^-1 to "pp"
% and also the POE E4*E5*E6 to "pp" knowing already Theta3-Theta2-Theta1,
% resulting exactly a Canonic problem PADEN-KAHAN-TWO, because the screws
% E6 does not affect "pp" & Th3-Th2-Th1 known (four solutions), the problem
% exp(E4^theta4)*exp(E5^theta5)*pp = pk2p ; with
% pk2p = exp(E3^Th3)^-1*exp(E2^Th2)^-1*exp(E1^Th1)^-1*noap*gs0^-1*pp 
% which by PADEN-KAHAN-TWO has none, one or two DOUBLE solutions:
% t31,t21,t11 to t41-t51 & t42-t52 ; t31,t22,t12 to t43-t53 & t44-t54
% t32,t23,t13 to t45-t55 & t46-t56 ; t32,t24,t14 to t47-t57 & t48-t58
%
t4t5 = zeros(8,2);
noapHst0ip = noap*(Hst0\[pp; 1]); 
for i = 1:2:7                     % for the 4 values of t3-t2-t1.
    pk2pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0ip;
    pk2pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk2pt;
    pk2p = pk2pt(1:3);
    t4t5(i:i+1,1:2) = PadenKahanPardosTwo(Twist(:,4),Twist(:,5),pp,pk2p); 
end
    Theta(:,4:5) = t4t5;          % put t4t5 values into Theta
%
% STEP2-4: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 (8 solutions),
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (eight solutions) we get t61...t68:
%
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:size(Theta,1)
    pk2pt = (expScrew([Twist(:,1);Theta(i,1)]))\noapHst0io;
    pk2pt = (expScrew([Twist(:,2);Theta(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta(i,3)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,4);Theta(i,4)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,5);Theta(i,5)]))\pk2pt;
    pk3p = pk2pt(1:3);
    Theta(i,6) = PadenKahanPardosOne(Twist(:,6), po, pk3p);
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