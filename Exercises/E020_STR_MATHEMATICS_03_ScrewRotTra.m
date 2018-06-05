%% Screw Theory for Robotics - MATHEMATICS - Exercise_03.
% Rigid Body Motion - HomogeneousTransformation  - Rotation + Translation  
% Transform a vector rt(-3,4,-11) expressed in coordinates of the T(OUVW)
% system, to its expression in coordinates of the reference system S(OXYZ).
% The system T(OUVW) is rotated pi/2 on the axis OX and then translated by 
% a vector the vector ps(8,-4,12), with respect to S(OXYZ).
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
%% E020_STR_MATHEMATICS_03_ScrewRotTra
clear;
clc;
rt = [-3; 4; -11; 1];
ps = [8; -4; 12];
theta1 = norm(ps);
omega1= ps/theta1;
xi1 = [omega1; 0; 0; 0];
theta2 = pi/2;
omega2 = [1; 0; 0];
xi2 = [0; 0; 0; omega2];
Et1 = expScrew([xi1; theta1]);
Et2 = expScrew([xi2; theta2]);
% "expScrew" by Dr. Pardos-Gotor ST24R.
Hst0 = eye(4);
rs = Et1 * Et2 * Hst0 * rt
