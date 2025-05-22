clc
%
% Run this file to initialize PID controllers for model 5 and 6
%

% Model 5 (LQR same as Ros/Gazebo) & 6 (PID + MPC)
%
% position controller
x_P = 1.5; 
x_I = 0;
x_D = 0;

y_P = 1.5;
y_I = 0;
y_D = 0.3;

z_P = 2.5;
z_I = 0;
z_D = 0;

% velocity controller
vy_P = 12;
vy_I = 0;
vy_D = 0;

% attitude controller
yaw_P = 1.5;
yaw_I = 0;
yaw_D = 0;

pitch_P = 10;
pitch_I = 0;
pitch_D = 0;

roll_P = 8;
roll_I = 0;
roll_D = 1;

% rates controller
p_P = 0.35;
p_I = 0;
p_D = 0;

r_P = 1;
r_I = 0;
r_D = 0;
