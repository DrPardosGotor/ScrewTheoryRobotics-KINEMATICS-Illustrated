%% Screw Theory for Robotics - MATHEMATICS - Exercise_01. 
% Rigid Body Motion - HomogeneousTransformation  - Rotation + Translation  
% Transform a vector rt(-3,4,-11) expressed in coordinates of the T(OUVW)
% system, to its expression in coordinates of the reference system S(OXYZ).
% The system T(OUVW) is rotated pi/2 on the axis OX and then translated by 
% a vector the vector ps(8,-4,12), with respect to S(OXYZ).
%
% Copyright (C) 2003-2018, by Dr. Jose M. Pardos-Gotor.
%
%% E30_HT_MATHEMATICS_RotTra  
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
