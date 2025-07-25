clc
%
% Run this file to initialize PID controllers for model 3 and 4
%

% Model 3 (PID) & 4 (PID + LQR)
%
% position controller
x_P = 1.8; 
x_I = 0;
x_D = 0.7;

y_P = 1.8;
y_I = 0;
y_D = 0.7;

z_P = 3;
z_I = 0;
z_D = 0;

% velocity controller
vx_P = 5;
vx_I = 0;
vx_D = 0;

vy_P = 5;
vy_I = 0;
vy_D = 0;

vz_P = 9;
vz_I = 0;
vz_D = 0;

% attitude controller
yaw_P = 1;
yaw_I = 0;
yaw_D = 0.3; 

pitch_P = 1.4;
pitch_I = 0;
pitch_D = 0.4;

roll_P = 1.5;
roll_I = 0;
roll_D = 0.3;
