%% Screw Theory - EXAMPLES Inverse Kinematics - Exercise_66.
% GANTRY robot - ABB IRB IRB6620LX - Check Two Ways to solve it.
%
% The goal of this exercise is to prove two diffeent ways
% Which solve the Inverse Kinematics problem by "Screw Theory Robotics"
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% for a TcP configuration POSE (rot+tra) as Hst into Dexterous Workspace.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...6. There can be up to 4 right solutions for this
% problem using this approach. WE TEST TWO DIFFERENT WAYS TO SOLVE IT:
% STEP2-1: The algorithm uses the PG1+PG4+PK2+PK1 subproblems cosecutively.
% STEP2-2: The algorithm uses the PG1+PK3+PK1+PK2+PK1 subproblems.
% (PK stands for Paden-Kahan and PG stands for Pardos-Gotor subproblems).
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% with Theta = [t11...t61; t12...t62; ...; t14...t64] and checking we get
% the same TcP configuration POSE (rot+tra) as Hst.
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
%% E060_STR_EXAMPLESInvKin_66_ABBIRB6620LX_Mat2Ways
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
%
% STEP2-1: The algorithm uses the PG1+PG4+PK2+PK1 subproblems cosecutively.
Theta_STR1 = zeros(4,6);
tic;
% STEP2-1: Calculate Theta1.
% With "pf" on the axis of E4, E5, E6. We apply (noap*hs0^-1) to "pf"
% Doing so we get Theta1 applying the Canonic problem PARDOS-ONE,
% because the screws E4,E5,E6 do not affect "pf" for being on their axes
% and the E2,E3 do not change the plane where "pf" moves (perpendicular to
% the axis of those screws, and so do not affect the calculation for Theta1
% resulting the problem "exp(E1^theta1)*pf = noap*hs0^-1*pf" by PARDOS-ONE
% which has one solution for t11.
noapHst0if = noap*(Hst0\[pf; 1]); pk1 = noapHst0if(1:3);
Theta_STR1(1,1) = PardosGotorOne(Twist(:,1), pf, pk1);
% prepare Theta for next calculation
Theta_STR1(2,:) = Theta_STR1(1,:);
%
% STEP2-1: Calculate Theta2 & Theta3.
% With "pf" on the axis of E4, E5, E6 we apply (noap*hs0^-1) to "pf" and
% the POE E1..E6 also to "pf" having already known the value for Theta1
% resulting exactly a Canonic problem PARDOS-FOUR, because the screws
% E4,E5,E6 do not affect "pf" and the E1 is known,resulting the problem
% exp(E2^theta2)*exp(E3^theta3)*pf = exp(E1^Th1)^-1*noap*gs0^-1*pf = pk1p
% which by PARDOS-FOUR has none, one or two DOUBLE solutions.
% t21-t31 & t22-t32 for each value of t11
%
E1inoapHst0if = (expScrew([Twist(:,1);Theta_STR1(1,1)]))\noapHst0if;
pk2 = E1inoapHst0if(1:3);
Theta_STR1(1:2,2:3) = PardosGotorFour(Twist(:,2),Twist(:,3),pf,pk2);
% prepare Theta for next calculation
Theta_STR1(3:4,:) = Theta_STR1(1:2,:); 
%
% STEP2-1: Calculate Theta4 & Theta5.
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
    pk3 = (expScrew([Twist(:,1);Theta_STR1(i,1)]))\noapHst0ip;
    pk3 = (expScrew([Twist(:,2);Theta_STR1(i,2)]))\pk3;
    pk3 = (expScrew([Twist(:,3);Theta_STR1(i,3)]))\pk3;
    pk3 = pk3(1:3);
    t4t5 = PadenKahanTwo(Twist(:,4),Twist(:,5),pp,pk3);
    Theta_STR1(i,4:5) = t4t5(1,:); Theta_STR1(i+2,4:5) = t4t5(2,:);
end
%
% STEP2-1: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 solutions,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (four solutions) we get t6:
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:4
    pk4 = (expScrew([Twist(:,1);Theta_STR1(i,1)]))\noapHst0io;
    pk4 = (expScrew([Twist(:,2);Theta_STR1(i,2)]))\pk4;
    pk4 = (expScrew([Twist(:,3);Theta_STR1(i,3)]))\pk4;
    pk4 = (expScrew([Twist(:,4);Theta_STR1(i,4)]))\pk4;
    pk4 = (expScrew([Twist(:,5);Theta_STR1(i,5)]))\pk4;
    pk4 = pk4(1:3);
    Theta_STR1(i,6) = PadenKahanOne(Twist(:,6), po, pk4);
end
%
Theta_STR1
%
tIK1 = round(toc*1000,1);
time_IK_STR1 = ['Time to solve IK Screw Theory ', num2str(tIK1),' ms']
%
% STEP2-2: The algorithm uses the PG1+PK3+PK1+PK2+PK1 subproblems.
Theta_STR2 = zeros(4,6);
tic;
% STEP2-2: Calculate Theta1.
% With "pf" on the axis of E4, E5, E6. We apply (noap*hs0^-1) to "pf"
% Doing so we get Theta1 applying the Canonic problem PARDOS-ONE,
% because the screws E4,E5,E6 do not affect "pf" for being on their axes
% and the E2,E3 do not change the plane where "pf" moves (perpendicular to
% the axis of those screws, and so do not affect the calculation for Theta1
% resulting the problem "exp(E1^theta1)*pf = noap*hs0^-1*pf" by PARDOS-ONE
% which has one solution for t11.
noapHst0if = noap*(Hst0\[pf; 1]); pkp = noapHst0if(1:3);
Theta_STR2(1,1) = PardosGotorOne(Twist(:,1), pf, pkp);
% prepare Theta for next calculation
Theta_STR2(2,1) = Theta_STR2(1,1);
%
% STEP2-2: Calculate Theta3 (alternative solution).
% With "pf" on the axis of E4, E5, E6 and "pk" on the axis of E2.
% We apply (exp(-E1^theta1)*noap*gs0^-1) to "pf" and take the norm of the
% diffence of that resulting point and "pk". Doing so we calculate Theta3
% applying the Canonic problem PADEN-KAHAN-THREE, because E4,E5,E6 do not 
% affect "pf" and E2 do not affect the norm of a vector with an end on "pk"
% resulting ||exp(E3^theta3)*pf-pk||=||exp(-E1^theta1)*noap*gs0^-1*pf-pk||
% which by PADEN-KAHAN-THREE has none, one or two solutions for t31 t32.
E1inoapHst0if = (expScrew([Twist(:,1);Theta_STR2(1,1)]))\noapHst0if;
pk1p = E1inoapHst0if(1:3); de = norm(pk1p - pk);
Theta_STR2(1:2,3)  = PadenKahanThree(Twist(:,3), pf, pk, de);
% prepare Theta for next calculation
Theta_STR2(3:4,:) = Theta_STR2(1:2,:);
%
% STEP2-2: Calculate Theta2 (alternative solution).
% With "pf" on axis E4,E5,E6, apply (exp(-E1^theta1)*noap*gs0^-1) to "pf"
% applying the Canonic problem PADEN-KAHAN-ONE, because E4,E5,E6 do not 
% affect "pf" and E3 is already known, resulting
% exp(E2^theta2)*exp(E3^theta3)*pf=exp(-E1^theta1)*noap*gs0^-1*pf equals
% exp(E2^theta2)*pf' = exp(-E1^theta1)*noap*gs0^-1*pf = pk1p
% which by PADEN-KAHAN-ONE has none or one solution for t21 t22.
for i = 1:2
    pfph = (expScrew([Twist(:,3);Theta_STR2(i,3)]))*[pf; 1];
    pfp = pfph(1:3);
    Theta_STR2(i,2) = PadenKahanOne(Twist(:,2), pfp, pk1p);
end
% prepare Theta for next calculation
Theta_STR2(3:4,2) = Theta_STR2(1:2,2);
%
% STEP2-2: Calculate Theta4 & Theta5.
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
    pk2pt = (expScrew([Twist(:,1);Theta_STR2(i,1)]))\noapHst0ip;
    pk2pt = (expScrew([Twist(:,2);Theta_STR2(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta_STR2(i,3)]))\pk2pt;
    pk2p = pk2pt(1:3);
    t4t5 = PadenKahanTwo(Twist(:,4),Twist(:,5),pp,pk2p);
    Theta_STR2(i,4:5) = t4t5(1,:); Theta_STR2(i+2,4:5) = t4t5(2,:);
end
%
% STEP2-2: Calculate Theta6.
% With "po" not in the axis of E6 apply E5^-1...*E1^-1*noap*gs0^-1 to "po"
% and applying E6 to "po" knowing already Theta5...Theta1 solutions,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E6^theta6)*po = pk3p ; with
% pk3p = exp(E5^Th5)^-1*...*exp(E1^Th1)^-1*noap*gs0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th5-Th4-Th3-Th2-Th1 known (four solutions) we get t6:
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:4
    pk2pt = (expScrew([Twist(:,1);Theta_STR2(i,1)]))\noapHst0io;
    pk2pt = (expScrew([Twist(:,2);Theta_STR2(i,2)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,3);Theta_STR2(i,3)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,4);Theta_STR2(i,4)]))\pk2pt;
    pk2pt = (expScrew([Twist(:,5);Theta_STR2(i,5)]))\pk2pt;
    pk3p = pk2pt(1:3);
    Theta_STR2(i,6) = PadenKahanOne(Twist(:,6), po, pk3p);
end
%
Theta_STR2
%
tIK2 = round(toc*1000,1);
time_IK_STR2 = ['Time to solve IK Screw Theory ', num2str(tIK2),' ms']
%
%
% STEP3: Test the different solutions applying ForwardKinemats to Robot
for i = 1:size(Theta_STR1,1)
    TwMagi = [Twist; Theta_STR1(i,:)];
    HstRi = ForwardKinematicsPOE(TwMagi);
    i
    noapi = HstRi * Hst0
end
for i = 1:size(Theta_STR2,1)
    TwMagi = [Twist; Theta_STR2(i,:)];
    HstRi = ForwardKinematicsPOE(TwMagi);
    i
    noapi = HstRi * Hst0
end
%
% Check that TcP POSE (rot+tra) as Hst is OK for all Theta values
%