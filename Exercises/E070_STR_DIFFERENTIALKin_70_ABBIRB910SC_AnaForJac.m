%% Screw Theory - DIFFERENTIAL Kinematics - Exercise_70.
% ABB IRB 910SC
%
% The goal of this exercise is to prove DIFFERENTIAL FORWARD KINEMATICS
% (Tcp Velocities) based on: ANALYTIC JACOBIAN
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% For checking this exercise:
% STEP1: Apply ForwardKinemats for the Robot with Screw Theory.
% STEP2: Apply ForwardKinemats with the direct formula known.
% STEP3: Solve the TcP velocities with the ANALYTIC JACOBIAN (obtained by 
% differentiation of the forward kinematics) and the joint velocities.
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
% http://www.preh
%
% CHANGES:
% Revision 1.1  2018/02/11 00:00:01
% General cleanup of code: help comments, see also, copyright
% references, clarification of functions.
%
%% E070_STR_DIFFERENTIALKin_70_ABBIRB910SC_AnaForJac
%
clear
clc
%
% Introduce the desired TcP pose necessary to evaluate Analytic Jacobian. 
Theta = [-pi/2 -pi/2 0 0]
% Introduce the desired Joint velocities.
Thetap = [0.35 -1.27 0 0]'
%
% Mechanical characteristics of the Robot:
l1 = 0.4; l2 = 0.25; l3 = 0.125;
po=[0;0;0]; pr=[l1;0;0]; pf=[l1+l2;0;0]; pp=[l1+l2;l3;0]; 
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [po pr pf pp];
Joint = ['rot'; 'rot'; 'tra'; 'rot'];
Axis = [AxisY AxisY -AxisY -AxisY];
Twist = zeros(6,4);
for i = 1:4
    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotX2tform(pi/2);
%
% STEP1: Apply ForwardKinemats to the Robot.
tic;
TwMag = [Twist; Theta]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noapSTR = HstR * Hst0
tSTR = round(toc*1000,1);
time_FK_STR = ['Time to solve FK Screw Theory ', num2str(tSTR),' ms']
%
% The Forward Kinematics for the robot are defined directly by:
tic;
t1=Theta(1); t2=Theta(2); t3=Theta(3); t4=Theta(4);
S1 = sin(t1); C1 = cos(t1); S2 = sin(t2); C2 = cos(t2);
S12 = sin(t1+t2); C12 = cos(t1+t2);
%
px = l2*C12+l1*C1;
py = l3-t3;
pz = -l2*S12-l1*S1;
al = pi/2;
be = Theta(1)+Theta(2)-Theta(4);
ga = 0;
%
noapD = rotZ2tform(ga)*rotY2tform(be)*rotX2tform(al);
noapD(1:3,4)= [px; py; pz]
tDIR = round(toc*1000,1);
time_FK_DIR = ['Time to solve FK Screw Theory ', num2str(tDIR),' ms']
%
% The ANALYTICAL JACOBIAN obtained by diffeerentiation of the FK.
Ja = [-l2*S12-l1*S1 -l2*S12 0   0;
                  0       0 -1  0;
      -l2*C12-l1*C1 -l2*C12  0  0;
                  0       0  0  0;
                  1       1  0 -1;
                  0       0  0  0];
% With the Forward Analytical JACOBIAN.
% Now we can calculate the velocity for the TcP xp, yp, zp, ap, bp, gp.
TcPpAJ = Ja*Thetap
%
%