%% Screw Theory for Robotics - MATHEMATICS - Exercise_02.
% Screw Rotation with exponential.  
% Transform a vector rt(3;2;1) expressed in coordinates of the mobil system
% T(OUVW), to its expression rs? in coordinates of the reference system
% S(OXYZ). The system T(OUVW) is rotated by an angle gamma(pi/4) with
% respect to the axis OZ of the system S(OXYZ).
%
% Using Screw Theory Functions from ST24R.
% by Dr. Pardos-Gotor ST24R "Screw Theory Toolbox for Robotics" MATLAB.
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
%% E020_STR_MATHEMATICS_02_ScrewRotation
clear;
clc;
gamma = pi/4;
W = [0 0 1];
rotm = expAxAng([W gamma]); % expAxAng implements Rodrigues' formula.
rt = [3; 2; 1];
rs = rotm * rt