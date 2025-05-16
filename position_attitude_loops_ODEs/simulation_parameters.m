clc
%
% Run this file to initialize parameters and trajectory reference
%
% Initial condition and reference defined as: [x_v y_v z_v roll pitch yaw]
%
% Coordinate frames defined as x forward, y left and z up
%
% [x_v y_v z_v] are expressed in vehicle frame
%

g = 9.81;

% UAV parameters
Lh = 0.2; % longitudinal distance btw rotors and pitch axis [m]
Lv = 0.2; % lateral distance btw rotors and roll axis [m]
m = 1.12; % mass [kg]
I = diag([0.018 0.035 0.051]); % inertia [kg/m^2]

% rotors parameters
kf = 65; % thrust to torque ratio of single rotor
Tmax = 4.5; % maximum thrust of single rotor

% reference & initial condition
initial = [0 0 0 0 0 0]'; % initial position
reference = [1 1.5 0.5 0 40 60]'; % reference
s_r = [1 1 1 1 1 1]; % use 0 for a step and 1 for a saturated ramp

initial(4) = initial(4)*pi/180;
initial(5) = initial(5)*pi/180;
initial(6) = initial(6)*pi/180;
initial_u = initial(2);
initial_f = [initial(1); initial(3:end)];

% debugging
delay_tilt = 0.004;
toll = 1e-12;
