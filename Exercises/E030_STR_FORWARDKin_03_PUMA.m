%% Screw Theory for Robotics - FORWARD Kinematics - Exercise_03.
% PUMA Type Manipulator (Generic Architecture)
%
% Calculate the Homogeneous Matrix transformation for the end-effector of
% a PUMA type robot of six Joints
% whose "Twist-Magnitude" parameters are defined by:
% AxisX = [1 0 0]; AxisY = [0 1 0]; AxisZ = [0 0 1]; 
% Points: pk = [0 2 0]; pr = [0 3 0]; pf = [0 5 0]; pp = [0 5 1];
% Axis = [AxisY; AxisX; AxisX; AxisY; AxisX; AxisZ];
% Point = [pk; pk; pr; pf; pf; pf];
% Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
% Mag = [pi p�/2 pi/3 pi/4 pi/5 pi/6];
%
% ans =
%
%   -0.8202   -0.0064   -0.5721   -0.5721
%   -0.1480   -0.9635    0.2230    0.4910
%   -0.5526    0.2676    0.7893   -1.2107
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
%% E030_STR_FORWARDKin_03_PUMA
%
AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
pk = [0 2 0]'; pr = [0 3 0]'; pf = [0 5 0]'; pp = [0 5 1]';
Axis = [AxisY AxisX AxisX AxisY AxisX AxisZ];
Point = [pk pk pr pf pf pf];
Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
Twist = Joint2Twist(Axis(:,1), Point(:,1), Joint(1,:));
for i = 2:size(Point,2)
    Twist = [Twist Joint2Twist(Axis(:,i), Point(:,i), Joint(i,:))];
end
Mag =[pi pi/2 pi/3 pi/4 pi/5 pi/6];
TwMag = [Twist; Mag];
HstR = ForwardKinematicsPOE(TwMag);
Hst0 = trvY2tform(pp(2))*trvZ2tform(pp(3));
Hst = HstR * Hst0
%