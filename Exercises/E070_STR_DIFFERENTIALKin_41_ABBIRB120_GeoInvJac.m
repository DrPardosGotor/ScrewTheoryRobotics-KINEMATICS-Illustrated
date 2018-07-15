%% Screw Theory - DIFFERENTIAL Kinematics - Exercise_41.
% ABB IRB 120
%
% The goal of this exercise is to solve for the Joint Velocities, taken
% the Tool pose velocities (i.e. TcP position and T frame rotation vel).
% The calculation is based on: Inverse GEOMETRIC JACOBIAN
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% FIST
% define the INPUTS are:
% VtS = the desired velocity for Tool Pose in spatial frame (S).
% Path = Trajectory in the plane X-Z defined by four points t1..t4.
% SECOND 
% For each point in the trajectory we need the joints magnitudes, Therefore
% we must solve the Inverse Kinematics problem for the robot in the poses.
% THIRD
% Calculate the TWISTS for the robot.
% Calculate the GEOMETRIC JACOBIAN.
% Solve the joint velocities for the trajectory with the inverse Jacobian
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
%% E070_STR_DIFFERENTIALKin_41_ABBIRB120_GeoInvJac
%
clear
clc
%
%
% the INPUTS are:
% VtS = the desired velocity for Tool Pose in spatial frame (S).
VtS = [0.1402 0 -0.2308 0 -0.92 0]';
% Path = Trajectory in the plane X-Z defined by four points t1..t4.
% t1..4 (6x1) = point in the trajectory defining for the Tool Pose the
% "Tpos" (3x1) position and "Trot" (3x1) rotation (order X+Y+Z).
t1 = [-0.25 0.125 0.4 pi/2 0 pi/2];
t2 = [-0.1 0.125 0.17 pi/2 0 pi/2];
t3 = [-0.0001 0.125 0.15 pi/2 0 pi/2];
t4 = [0.1 0.125 -0.17 pi/2 0 pi/2];
Path = [t1; t2; t3; t4];
%
%
% For each point in the trajectory we need the joints magnitudes, as the 
% Geometric Jacobian is a valued matrix. Therefore we must solve the 
% Inverse Kinematics problem for the robot in the t1..t4 poses.
% We know that in general there are 8 solutions for each robot pose, then
% we select only one valid solution (for instance the #5).
% Theta = matrix where to save all sets of joint magnitudes for each pose.
Theta = zeros(size(Path,1),6);
for i = 1:size(Theta)
    Theta(i,:) = Fcn_ABBIRB120_InverseKinematics([Path(i,:) 5]);
end
%
%
% Mechanical characteristics of the Robot: for getting the robot TWISTS.
po=[0;0;0]; pk=[0;0.290;0]; pr=[0;0.560;0];
pf=[0.302;0.630;0]; pp=[0.302;0.470;0];
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
Point = [pk pk pr pf pf pp];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Axis = [AxisY AxisZ AxisZ AxisX AxisZ -AxisY];
Twist = zeros(6,6);
for i = 1:6
    Twist(:,i) = Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:));
end
%
% 
% ThetapGJ (6xn)= matrix with the results, these are the values for the
% joints velocities for each point in the trajectory of the Tool. Results
% are stored by columns.
ThetapGJ = zeros(size(Theta,2),size(Theta,1));
%
tic;
for i = 1:size(Theta)
% The MANIPULATOR JACOBIAN by DEFINITION.
TwMag = [Twist; Theta(i,:)];
JstS = Smanipulatorjacobian(TwMag);
%
% The Velocity for the Joints of the Robot with inverse GEOMETRIC JACOBIAN.
ThetapGJ(:,i) = JstS\[VtS(1:3)-skew(VtS(4:6))*Path(i,1:3)'; VtS(4:6)];
%
end
tIGJ = round(toc*1000,1);
time_IK_GJ = ['Time differential inverse kinematics ', num2str(tIGJ),' ms']
ThetapGJ
%