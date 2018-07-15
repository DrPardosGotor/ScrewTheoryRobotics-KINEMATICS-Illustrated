%% Screw Theory - DIFFERENTIAL Kinematics - Exercise_40.
% ABB IRB ¡120
%
% The goal of this exercise is to prove the forward (Tcp Velocities)
% calculation, based on: GEOMETRIC JACOBIAN
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
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
%% E070_STR_DIFFERENTIALKin_40_ABBIRB120_GeoForJac
%
clear
clc
%
Theta = [-2.1294 -0.9092 0.0977 0 0.8116 -2.1294];
Thetap = [-0.0073 0.8807 -1.6060 -0.0001 0.7253 0.9128]';
%
% Mechanical characteristics of the IRB120 Robot:
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
Hst0 = trvX2tform(pp(1))*trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst0 = Hst0*rotX2tform(pi/2)*rotZ2tform(pi/2);
%
% Apply ForwardKinemats to the Robot.
TwMag = [Twist; Theta]; % assign the rand values to joint Theta magnitudes.
HstR = ForwardKinematicsPOE(TwMag);
noap = HstR * Hst0
%
% The MANIPULATOR JACOBIAN by DEFINITION.
JstS = Smanipulatorjacobian(TwMag)
%
% Differential Forward Kinematics with Jacobian by DEFINITION.
% Spatial Velocity in terms of the screw theory VstS.
VstS = JstS*Thetap
%TcPpJd = [Twist2tform(VstS)*noap(:,4); VstS(4:6)]
% Tool Velocity in the Spatial frame VtS.
VtS = [VstS(1:3)+skew(VstS(4:6))*noap(1:3,4); VstS(4:6)]
%