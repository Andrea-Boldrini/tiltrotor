clc
%
% Run this file to initialize PID controllers
%
% "var" has to be set to: 3 for model_3
%                         4 for model_4
%                         5 for model_5 and model_6
%                         7 for model_7
%

var = 5;

switch var

case 7 % MPC scheme updated (model_7)
% position controller
x_P = 1.5; 
x_I = 0;
x_D = 0;

y_P = 0.8;
y_I = 0;
y_D = 0;

z_P = 2.5;
z_I = 0;
z_D = 0;

% velocity controller
vy_P = 12;
vy_I = 0;
vy_D = 0;

vz_P = 9;
vz_I = 1.5;
vz_D = 0;

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

% rate controller
p_P = 0.35;
p_I = 0;
p_D = 0;

case 5 % LQR scheme updated and MPC (model_5 & model_6)
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

vz_P = 9;
vz_I = 0;
vz_D = 0;

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

q_P = 10;
q_I = 0;
q_D = 0;

r_P = 1;
r_I = 0;
r_D = 0;

case 4 % LQR scheme (model_4)
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
yaw_D = 0.5; 

pitch_P = 1.4;
pitch_I = 0;
pitch_D = 0.4;

roll_P = 1.5;
roll_I = 0;
roll_D = 0.3;

case 3 % PID scheme (model_3)
% position controller
x_P = 6; 
x_I = 0;
x_D = 1.5;

y_P = 1.5;
y_I = 0;
y_D = 0.3;

z_P = 3;
z_I = 0;
z_D = 0;

% velocity controller
vx_P = 10;
vx_I = 0;
vx_D = 0;

vy_P = 12;
vy_I = 0;
vy_D = 0;

vz_P = 8;
vz_I = 0;
vz_D = 0;

% attitude controller
yaw_P = 1.5;
yaw_I = 0;
yaw_D = 0; 

pitch_P = 2;
pitch_I = 0.05;
pitch_D = 0;

roll_P = 8;
roll_I = 0;
roll_D = 1;

% rate controller
r_P = 1;
r_I = 0;
r_D = 0; 

q_P = 1.5;
q_I = 0;
q_D = 0;

p_P = 0.35;
p_I = 0;
p_D = 0;

end