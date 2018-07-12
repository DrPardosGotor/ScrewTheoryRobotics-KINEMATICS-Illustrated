%% Screw Theory for Robotics - Simulation - Exercise_43.
% ABB IRB120 Robot ToolDown.
%
% The goal of this exercise is to compare the performance between
% ST24R "Screw Theory Toolbox for Robotics" by Dr. Pardos-Gotor.
% and
% RST "Robotics System Toolbox" by MATHWORKS
% for solving the Inverse Kinematics of the IRB120:
 
% For the ST24R we use:
% function ThetaOut = Fcn_IRB120_Kinematics_ToolDown(u)
% The inputs "u" are composed by the following vectors.
% "traXYZ" (3x1) desired translations for the TcP (noap - "p" goal).
% "rotXYZ" (3x1) desired rotations for TcP (noap - "noa" goal order X+Y+Z).
% "ti" (1x1) "Theta index" for choosing ONE out of the 8 possible results.
% "ThetaOut" (t1..t6)are the magnitudes solution for the Robot Joints1..6.
%
% For the RST we use:
% The Broyden-Fletcher-Goldfarb-Shanno (BFGS) gradient projection algorithm
% iterative, gradient-based optimization methods. 
%
% For checking the quality of this IK solution, this exercise has 3 steps:
% STEP1: Apply ForwardKinemats for the Robot for "whatever" Mag Theta1...6
% so getting a feasible TcP configuration (rot+tra) as Hst.
% STEP2: Calculate the IK solutions by ST24R and RST getting
% the magnitud Theta1...6. There can be up to 8 right solutionS
% STEP3: Test the different solutions applying ForwardKinemats to Robot
% and checking we get the same TcP POSE configuration (rot+tra) as Hst.
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
%% E060_STR_EXAMPLESInvKin_47_ABBIRB120_ST24RandRST
%
clear
clc
for i = 1:6
    Mag(i) = (rand-rand)*pi; % Magnitudes Theta1-Theta6 for testing.
end
%Mag = [0.1 0.1 0.1 0.1 0.1 0.1];
MagST24R = Mag;
%% Application "ST24R - Screw Theory Robotics" for Kinematics.
%
% Mechanical characteristics of the IRB120 Robot:
po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
pf=[0.302;0.630;0]; pp=[0.302;0.470;0];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pk pr pf pf pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisY AxisZ AxisZ AxisX AxisZ -AxisY]; % Attention Axis6 is neg.
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
end
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotX2tform(pi/2)*rotY2tform(0)*rotZ2tform(pi/2);
%
% STEP1: Apply ForwardKinemats to the Robot.
TwMag = [Twist; MagST24R]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noapST24R = HstR * Hst0
%
% STEP2: Calculate the IK solutions Theta using the SCREW THEORY techniques
% and basically the PADEN-KAHAN-PARDOS Canonic Subproblems.
traXYZ = noapST24R(1:3,4);
rotXYZ = tform2eul(noapST24R,'XYZ');
ti = 5; % we choose only the first solution for ThetaOut
u = [traXYZ', rotXYZ, ti];
tic;
ThetaST24R = Fcn_ABBIRB120_InverseKinematics_mex(u)
tIKST24R = round(toc*1000000,0);
time_IK_ST24R = ['Time to solve IK Screw Theory ', num2str(tIKST24R),' μs']
%
% STEP3: Test the solution applying ForwardKinemats to Robot
TwMag = [Twist; ThetaST24R];
HstR = ForwardKinematicsPOE(TwMag);
TcpST24R = HstR * Hst0
%
%% Application "RST - Robotics System Toolbox - MATLAB" - for Kinematics.
%
% Mechanical characteristics of the ABB IRB120 Robot.
% ATTENTION! because for the RST standard the coordinate system of the base
% has the "Z" axis in the vertical direction (instead of the "Y" axis)
% Therefore the configuration of the robot has other coordinates.
po=[0;0;0]; pk=[0;0;0.290]; pr=[0;0;0.560];
pf=[0.302;0;0.630]; pp=[0.302;0;0.470];
%
tformj1 = rotX2tform(pi/2);
tformj2 = trvY2tform(pk(3))*rotX2tform(-pi/2);
tformj3 = rotX2tform(pi/2);
tformj4 = trvY2tform(pr(3)-pk(3));
tformj5 = trvX2tform(pf(1))*trvY2tform(pf(3)-pr(3))*rotY2tform(pi/2);
tformj6 = rotY2tform(-pi/2);
tformj7 = trvY2tform(pp(3)-pf(3))*rotX2tform(pi/2)*rotZ2tform(pi/2);
%
% Create a rigid body tree object to build the robot.
robot = robotics.RigidBodyTree;
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','revolute');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');
setFixedTransform(jnt1,tformj1);
setFixedTransform(jnt2,tformj2);
setFixedTransform(jnt3,tformj3);
setFixedTransform(jnt4,tformj4);
setFixedTransform(jnt5,tformj5);
setFixedTransform(jnt6,tformj6);
setFixedTransform(jnt7,tformj7);
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')
addBody(robot,body7,'body6')
%
% STEP1: Apply ForwardKinemats to the Robot.
% Verify that your robot was built properly by using the |showdetails| or
% showdetails(robot);
% show(robot);
% getTransform permits to evaluate the Forward Kinematics.
joints = {'jnt1','jnt2','jnt3','jnt4','jnt5','jnt6','jnt7'};
MagRST = {0,Mag(1),Mag(2),Mag(3),Mag(4),Mag(5),Mag(6)};
FKRST = struct('JointName', joints,'JointPosition', MagRST);
TcpRST = getTransform(robot,FKRST,'body7','body1');
%
% STEP2: Calculate the IK solutions with Robtics System Toolbox
tic;
noapRSTbase = getTransform(robot,FKRST,'body7','base');
weights = [1e-6,1e-6,1e-6,1e-6,1e-6,1e-6];
qinitial = robot.homeConfiguration;
IKRST = robotics.InverseKinematics('RigidBodyTree', robot);
IKRST.SolverParameters.MaxIterations= 15000;
[IKRSTsol,solinfo] = IKRST('body7',noapRSTbase,weights,qinitial);
ThetaRST = {IKRSTsol(1).JointPosition; IKRSTsol(2).JointPosition;
            IKRSTsol(3).JointPosition; IKRSTsol(4).JointPosition;
            IKRSTsol(5).JointPosition; IKRSTsol(6).JointPosition;
            IKRSTsol(7).JointPosition}';
ThetaRSTn = cell2mat(ThetaRST);
ThetaRSTn6 = ThetaRSTn(2:7)
tIKRST = round(toc*1000,0);
time_IK_RST = ['Time to solve IK RS Toolbox ', num2str(tIKRST),' ms']
%
% STEP3: Test the solution applying ForwardKinemats to Robot
FKRST = struct('JointName', joints,'JointPosition', ThetaRST);
TcpRST = getTransform(robot,FKRST,'body7','body1')
%