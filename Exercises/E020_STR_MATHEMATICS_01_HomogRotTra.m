%% Screw Theory for Robotics - MATHEMATICS - Exercise_01. 
% Rigid Body Motion - HomogeneousTransformation  - Rotation + Translation  
% Transform a vector rt(-3,4,-11) expressed in coordinates of the T(OUVW)
% system, to its expression in coordinates of the reference system S(OXYZ).
% The system T(OUVW) is rotated pi/2 on the axis OX and then translated by 
% a vector the vector ps(8,-4,12), with respect to S(OXYZ).
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
%% E020_MATHEMATICS_01_HomogRotTra  
clear;
clc;
rt = [-3; 4; -11; 1];
alfa = pi/2;
Ca = cos(alfa);
Sa = sin(alfa);
Hx = [1 0 0 0; 0 Ca -Sa 0; 0 Sa Ca 0; 0 0 0 1];
ps = [8; -4; 12; 1];
Hp = eye(4);
Hp(:,4) = ps;
rs = Hp * Hx * rt
