%% Screw Theory for Robotics - FORWARD Kinematics - Exercise_05.
% STANFORD Type Manipulator (Generic Architecture)
%
% Calculate the Homogeneous Matrix transformation for the end-effector of
% a STANFORD type robot of six Joints
% whose "Twist-Magnitude" parameters are defined by:
% AxisX = [1 0 0]; AxisY = [0 1 0]; AxisZ = [0 0 1]; 
% Points: po = [0 0 0]; pk = [0 2 0]; pf = [0 5 0]; pp = [0 5 1];
% Axis = [AxisY; AxisX; AxisY; AxisY; AxisX; AxisZ];
% Point = [pk; pk; pk; pf; pf; pf];
% Joint = ['rot'; 'rot'; 'tra'; 'rot'; 'rot'; 'rot'];
% Mag = [pi p�/2 3 pi/4 pi/5 pi/6];
%
% ans =
%
%   -0.8202   -0.0064   -0.5721   -0.5721
%    0.4046   -0.7135   -0.5721    1.4279
%   -0.4045   -0.7006    0.5878   -5.4122
%         0         0         0    1.0000
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
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
%% E030_STR_FORWARDKin_05_STANFORD
%
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]'; 
pk = [0 2 0]'; pf = [0 5 0]'; pp = [0 5 1]';
Axis = [AxisY AxisX AxisY AxisY AxisX AxisZ];
Point = [pk pk pk pf pf pf];
Joint = ['rot'; 'rot'; 'tra'; 'rot'; 'rot'; 'rot'];
Twist = Joint2Twist(Axis(:,1), Point(:,1), Joint(1,:));
for i = 2:size(Point,2)
    Twist = [Twist Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:))];
end
Mag =[pi pi/2 3 pi/4 pi/5 pi/6];
TwMag = [Twist; Mag];
HstR = ForwardKinematicsPOE(TwMag);
Hst0 = trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst = HstR * Hst0
%