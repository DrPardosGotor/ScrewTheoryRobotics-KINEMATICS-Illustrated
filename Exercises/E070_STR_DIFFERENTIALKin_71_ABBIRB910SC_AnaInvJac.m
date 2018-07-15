%% Screw Theory - DIFFERENTIAL Kinematics - Exercise_71.
% ABB IRB 910SC
%
% The goal of this exercise is to prove the inverse (Joint Velocities)
% calculation, based on: ANALYTIC JACOBIAN
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
%% E070_STR_DIFFERENTIALKin_71_ABBIRB910SC_AnaInvJac
%
clear
clc
%
Theta = [-pi/2 -pi/2 0 0; -pi/2 -2.7 0 0; -pi/2 -3.141 0 0; pi/2 -2.7 0 0]
TcPp = [0.14 0 -0.23]'   % the velocity for TcP Position.
ThetapAJ = zeros(3,size(Theta,1));
%
% Mechanical characteristics of the Robot:
l1 = 0.4; l2 = 0.25; l3 = 0.125;
%
for i = 1:size(Theta)
t1=Theta(i,1); t2=Theta(i,2); t3=Theta(i,3); t4=Theta(i,4);
S1 = sin(t1); C1 = cos(t1); S2 = sin(t2); C2 = cos(t2);
S12 = sin(t1+t2); C12 = cos(t1+t2);
%
% The Inverse Analytical JACOBIAN for only the TcP POSITION.
InJaPo = [-l2*C12 0 l2*S12; l2*C12+l1*C1 0 -l2*S12-l1*S1; 0 l1*l2*S2 0];
InJaPo = -InJaPo/(l1*l2*S2);
%
% The Velocity for the Joints of the Robot is:
ThetapAJ(:,i) = InJaPo*TcPp;
end
ThetapAJ
%