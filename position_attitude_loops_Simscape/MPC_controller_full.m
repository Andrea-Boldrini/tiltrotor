clc
%
% Run this file to generate MPC controller
%

% model parameters
mass = 1.12;
g = 9.81;
I_y = 0.035; 
I_x = 0.018;
I_z = 0.051;

% linearized continuous system
A = zeros(12,12);
A(1,2) = 1;           % dx/dt = vx
A(3,4) = 1;           % dy/dt = vy
A(4,7) = -g;          % dvy/dt = -g * phi
A(5,6) = 1;           % dz/dt = vz
A(7,8) = 1;           % dphi/dt = omega_phi
A(9,10) = 1;          % dtheta/dt = omega_theta
A(11,12) = 1;         % dpsi/dt = omega_psi

B = zeros(12,5);
B(2,1) = 1/mass;      % Fx affects vx
B(6,2) = 1/mass;      % Fz affects vz
B(8,3) = 1/I_x;       % roll torque
B(10,4) = 1/I_y;      % pitch torque
B(12,5) = 1/I_z;      % yaw torque

B_d = zeros(12, 6);
B_d(2,1) = 1/mass;   % vx disturbance
B_d(4,2) = 1/mass;   % vy disturbance
B_d(6,3) = 1/mass;   % vz disturbance
B_d(8,4) = 1/I_x;   % roll rate disturbance
B_d(10,5) = 1/I_y;  % pitch rate disturbance
B_d(12,6) = 1/I_z;  % yaw rate disturbance
B_aug = [B, B_d];
C = eye(12);
sys_c = ss(A, B_aug, C, 0);

% discretization
T = 10; % time horizon
N = 150; % number of time steps (without t0) - N=40 for ODEs, N=150 for Simscape
Ts = T / N; % sampling time

sys_d = c2d(sys_c, Ts, 'zoh');

input_names = {'u1','u2','u3', 'u4', 'u5', 'd1','d2','d3', 'd4', 'd5','d6'};
sys_d.InputName = input_names;
sys_d = setmpcsignals(sys_d, 'MV', 1:5, 'MD', 6:11);

% controller initialization
mpcobj = mpc(sys_d, Ts);

mpcobj.PredictionHorizon = 20;
mpcobj.ControlHorizon = 5;

mpcobj.Weights.ManipulatedVariables = [0.1 0.01 1 0.1 1];
mpcobj.Weights.ManipulatedVariablesRate = [0.01 0.01 0.01 0.01 0.01];
mpcobj.Weights.OutputVariables = [1 0.55 1 0.55 1 0.55 1 0.55 1 0.55 1 0.55];

% Optional constraints
mpcobj.MV(1).Min = -10; mpcobj.MV(1).Max = 10;
mpcobj.MV(2).Min = -10; mpcobj.MV(2).Max = 10;
mpcobj.MV(3).Min = -5;  mpcobj.MV(3).Max = 5;
mpcobj.MV(4).Min = -5;  mpcobj.MV(4).Max = 5;
mpcobj.MV(5).Min = -5;  mpcobj.MV(5).Max = 5;

%mpcobj.OV(7).Min = -10*pi/180; mpcobj.OV(7).Max = 10*pi/180;

assignin('base', 'mpcobj', mpcobj);