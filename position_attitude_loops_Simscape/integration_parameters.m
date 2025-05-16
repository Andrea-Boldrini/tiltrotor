clc
%
% Run this file to initialize PID controllers and simulation parameters
%
% Reference defined as: [x_v y_v z_v roll pitch yaw]
%
% Coordinate frames defined as x forward, y left and z up
%
% [x_v y_v z_v] are expressed in vehicle frame
%

g = 9.81;

% MPC scheme updated (model_7)
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

% % LQR scheme updated and MPC (model_5 & model_6)
% % position controller
% x_P = 1.5; 
% x_I = 0;
% x_D = 0;
% 
% y_P = 1.5;
% y_I = 0;
% y_D = 0.3;
% 
% z_P = 2.5;
% z_I = 0;
% z_D = 0;
% 
% % velocity controller
% vy_P = 12;
% vy_I = 0;
% vy_D = 0;
% 
% vz_P = 9;
% vz_I = 0;
% vz_D = 0;
% 
% % attitude controller
% yaw_P = 1.5;
% yaw_I = 0;
% yaw_D = 0;
% 
% pitch_P = 10;
% pitch_I = 0;
% pitch_D = 0;
% 
% roll_P = 8;
% roll_I = 0;
% roll_D = 1;
% 
% % rates controller
% p_P = 0.35;
% p_I = 0;
% p_D = 0;
% 
% q_P = 10;
% q_I = 0;
% q_D = 0;
% 
% r_P = 1;
% r_I = 0;
% r_D = 0;

% % LQR scheme (model_4)
% % position controller
% x_P = 1.8; 
% x_I = 0;
% x_D = 0.7;
% 
% y_P = 1.8;
% y_I = 0;
% y_D = 0.7;
% 
% z_P = 3;
% z_I = 0;
% z_D = 0;
% 
% % velocity controller
% vx_P = 5;
% vx_I = 0;
% vx_D = 0;
% 
% vy_P = 5;
% vy_I = 0;
% vy_D = 0;
% 
% vz_P = 9;
% vz_I = 0;
% vz_D = 0;
% 
% % attitude controller
% yaw_P = 1;
% yaw_I = 0;
% yaw_D = 0.5; 
% 
% pitch_P = 1.4;
% pitch_I = 0;
% pitch_D = 0.4;
% 
% roll_P = 1.5;
% roll_I = 0;
% roll_D = 0.3;

% % PID scheme (model_3)
% % position controller
% x_P = 6; 
% x_I = 0;
% x_D = 1.5;
% 
% y_P = 1.5;
% y_I = 0;
% y_D = 0.3;
% 
% z_P = 3;
% z_I = 0;
% z_D = 0;
% 
% % velocity controller
% vx_P = 10;
% vx_I = 0;
% vx_D = 0;
% 
% vy_P = 12;
% vy_I = 0;
% vy_D = 0;
% 
% vz_P = 8;
% vz_I = 0;
% vz_D = 0;
% 
% % attitude controller
% yaw_P = 1.5;
% yaw_I = 0;
% yaw_D = 0; 
% 
% pitch_P = 2;
% pitch_I = 0.05;
% pitch_D = 0;
% 
% roll_P = 8;
% roll_I = 0;
% roll_D = 1;
% 
% % rate controller
% r_P = 1;
% r_I = 0;
% r_D = 0; 
% 
% q_P = 1.5;
% q_I = 0;
% q_D = 0;
% 
% p_P = 0.35;
% p_I = 0;
% p_D = 0;

% UAV parameters
Lh = 0.2; % longitudinal distance btw rotors and pitch axis [m]
Lv = 0.2; % lateral distance btw rotors and roll axis [m]
m = 1.12; % mass of UAV
J = diag([0.018 0.035 0.051]); % inertia [kg/m^2]

% rotors parameters
kf = 65; % thrust to torque ratio of single rotor
Tmax = 5.5; % maximum thrust of single rotor

% debugging
delay_tilt = 0.004;
toll = 1e-12;

% reference & initial condition
reference = [1 1 1 0 30 40]'; % reference
s_r = [1 1 1 1 1 1]; % use 0 for a step and 1 for a saturated ramp

% external wrench (just for model_6)
a = 6;
%ext_force = zeros(6,1);
ext_force = [1 0 0.6 0.1 0.4 0.1]'; % max tau_roll = 0.2, max_tau_yaw = 0.1;

% damping/friction coefficients (empirical)
C2 = 0.15; % coefficient of damping/friction (0°)
C0 = 0.2; % coefficient of damping/friction (10°)
Cm = 0.105; % coefficient of damping/friction (30°)
C1 = 0.165; % coefficient of damping/friction (50°)

