%% Screw Theory - EXAMPLES Inverse Kinematics - Exercise_76.
% SCARA robot - ABB IRB 910SC - Check Two Ways to solve it.
%
% The goal of this exercise is to prove Mathematics in two diffeent ways
% Which solve the Inverse Kinematics problem by "Screw Theory Robotics"
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...4
% for a TcP configuration POSE (rot+tra) as Hst into Dexterous Workspace.
% STEP2: Calculate the IK solutions by SCREW THEORY management getting
% the magnitud Theta1...4. There can be up to 2 right solutions for this
% problem using this approach. WE TEST TWO DIFFERENT WAYS TO SOLVE IT:
% STEP2-1: The algorithm uses the PG1+PG4+PK1 subproblems cosecutively.
% STEP2-2: The algorithm uses the PG1+PK3+PK1+PK1 subproblems.
% (PK stands for Paden-Kahan and PG stands for Pardos-Gotor subproblems).
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% with Theta = [t11...t41; t12...t42] and checking we get
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
%% E060_STR_EXAMPLESInvKin_76_ABBIRB910SC_Mat2Ways
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
% STEP2-1: The algorithm uses the PG1+PG4+PK1 subproblems cosecutively.
Theta_STR1 = zeros(2,6);
tic;
%
% STEP2-1: Calculate Theta3.
% With "pf" on the axis of E4. We apply (noap*hs0^-1) to "pf"
% doing so we get Theta3 applying the Canonic problem PARDOS-ONE,
% because the screws E4 do not affect "pf" for being on its axis
% and the E1,E2 do not change the plane where "pf" moves (perpendicular to
% the axis of those screws, and so do not affect the calculation for Theta3
% resulting the problem "exp(E3^theta3)*pf = noap*hst0^-1*pf" by PARDOS-ONE
% which has one solution for t31.
pkp = noap*(Hst0\[pf; 1]);
Theta_STR1(1,3) = PardosGotorOne(Twist(:,3), pf, pkp(1:3));
% prepare Theta for next calculation
Theta_STR1(2,3) = Theta_STR1(1,3);
%
% STEP2-1: Calculate Theta1 & Theta2.
% With "pf" on the axis of E4, we apply (noap*hs0^-1) to "pf" and
% the POE E1..E4 also to "pf" having already known the value for Theta3
% resulting exactly a Canonic problem PARDOS-FOUR, because the screw
% E4 do not affect "pf" and the E3 is known,resulting the problem
% exp(E1^theta1)*exp(E2^theta2)*exp(E3^theta3)*pf = 
% exp(E1^theta1)*exp(E2^theta2)*pfp = noap*hst0^-1*pf = pkp
% which by PARDOS-FOUR has none, one or two DOUBLE solutions.
% t11-t21 & t12-t22 for each value of t31
%
pfp = expScrew([Twist(:,3);Theta_STR1(1,3)])*[pf; 1];
Theta_STR1(1:2,1:2) = PardosGotorFour(Twist(:,1),Twist(:,2),pfp(1:3),pkp(1:3));
%
% STEP2-1: Calculate Theta4.
% With "po" not in the axis of E4 apply E3^-1...*E1^-1*noap*hst0^-1 to "po"
% and applying E4 to "po" knowing already Theta3...Theta1 solutions,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E4^theta4)*po = pk3p ; with
% pk3p = exp(E3^Th3)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th3-Th2-Th1 known (two solutions) we get t4:
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:2
    pk3p = (expScrew([Twist(:,1);Theta_STR1(i,1)]))\noapHst0io;
    pk3p = (expScrew([Twist(:,2);Theta_STR1(i,2)]))\pk3p;
    pk3p = (expScrew([Twist(:,3);Theta_STR1(i,3)]))\pk3p;
    Theta_STR1(i,4) = PadenKahanOne(Twist(:,4), po, pk3p(1:3));
end
%
Theta_STR1
%
tIK1 = round(toc*1000,1);
time_IK_STR1 = ['Time to solve IK Screw Theory ', num2str(tIK1),' ms']
%
% STEP2-2: The algorithm uses the PG1+PK3+PK1+PK1 subproblems.
Theta_STR2 = zeros(2,6);
tic;
% STEP2-2: Calculate Theta3.
% With "pf" on the axis of E4. We apply (noap*hs0^-1) to "pf"
% doing so we get Theta3 applying the Canonic problem PARDOS-ONE,
% because the screws E4 do not affect "pf" for being on its axis
% and the E1,E2 do not change the plane where "pf" moves (perpendicular to
% the axis of those screws, and so do not affect the calculation for Theta3
% resulting the problem "exp(E3^theta3)*pf = noap*hst0^-1*pf" by PARDOS-ONE
% which has one solution for t31.
pkp = noap*(Hst0\[pf; 1]);
Theta_STR2(1,3) = PardosGotorOne(Twist(:,3), pf, pkp(1:3));
% prepare Theta for next calculation
Theta_STR2(2,:) = Theta_STR2(1,:);
%
% STEP2-2: Calculate Theta2 (alternative solution).
% With "pf" on the axis of E4 and "po" on the axis of E1.
% we apply noap*hst0^-1 to "pf" and take the norm of the diffence of that
% resulting point and "po". Doing so we calculate Theta2
% applying the Canonic problem PADEN-KAHAN-THREE, because E4 do not 
% affect "pf" and E1 do not affect the norm of a vector with an end on "po"
% resulting ||exp(E2^theta2)*exp(E3^theta3)*pf-po||=
% ||exp(E2^theta2)*pfp-po|| = ||noap*hst0^-1*pf-po|| = ||pkp-po|| = delta
% which by PADEN-KAHAN-THREE has none, one or two solutions for t21 t22.
%
de = norm(pkp(1:3) - po);
pfp = expScrew([Twist(:,3);Theta_STR2(1,3)])*[pf; 1];
Theta_STR2(1:2,2)  = PadenKahanThree(Twist(:,2), pfp(1:3), po, de);
%
% STEP2-2: Calculate Theta1 (alternative solution).
% With "pf" on axis E4 apply noap*hst0^-1) to "pf"
% applying the Canonic problem PADEN-KAHAN-ONE, because E4 do not 
% affect "pf" and E2, E3 are already known, resulting
% exp(E1^theta1)*exp(E2^theta2)*exp(E3^theta3)*pf = 
% exp(E1^theta1)*pf2p = noap*hst0^-1*pf = pkp
% which by PADEN-KAHAN-ONE has none or one solution for t11.
for i = 1:2
    pf2p = expScrew([Twist(:,2);Theta_STR2(i,2)])*pfp;
    Theta_STR2(i,1) = PadenKahanOne(Twist(:,1), pf2p(1:3), pkp(1:3));
end
%
% STEP3: Calculate Theta4.
% With "po" not in the axis of E4 apply E3^-1...*E1^-1*noap*hst0^-1 to "po"
% and applying E4 to "po" knowing already Theta3...Theta1 solutions,
% resulting exactly a Canonic problem PADEN-KAHAN-ONE, the problem:
% exp(E4^theta4)*po = pk3p ; with
% pk3p = exp(E3^Th3)^-1*...*exp(E1^Th1)^-1*noap*hst0^-1*po 
% which by PADEN-KAHAN-ONE has none or one solution. Then for all
% Th3-Th2-Th1 known (two solutions) we get t4:
noapHst0io = noap*(Hst0\[po; 1]);
for i = 1:2
    pk3p = (expScrew([Twist(:,1);Theta_STR2(i,1)]))\noapHst0io;
    pk3p = (expScrew([Twist(:,2);Theta_STR2(i,2)]))\pk3p;
    pk3p = (expScrew([Twist(:,3);Theta_STR2(i,3)]))\pk3p;
    Theta_STR2(i,4) = PadenKahanOne(Twist(:,4), po, pk3p(1:3));
end
%
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