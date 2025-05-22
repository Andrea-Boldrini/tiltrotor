clc
%
% Run this file to initialize simulation parameters
%
% Reference defined as: [x_v y_v z_v roll pitch yaw]
%
% Coordinate frames defined as x forward, y left and z up
%
% [x_v y_v z_v] are expressed in vehicle frame
%

g = 9.81;

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

