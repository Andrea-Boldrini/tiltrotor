clc
%
% Run this file to generate MPC controller
%

% model parameters
mass = 1.12;
g = 9.81;
I_y = 0.035; %I_x = 0.018 and I_z = 0.051

% linearized continuous system
A = [0 1 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0];
B = [0 0 0;
    1/mass 0 0;
    0 0 0;
    0 1/mass 0;
    0 0 0;
    0 0 1/I_y];
B_aug = [B zeros(size(B,1),size(B,2))];
B_aug(2,size(B,2)+1) = 1/mass;
B_aug(4,size(B,2)+2) = 1/mass;
B_aug(6,size(B,2)+3) = 1/I_y;
C = eye(6);
sys_c = ss(A, B_aug, C, 0);

% discretization
T = 10; % time horizon
N = 150; % number of time steps (without t0) - N=40 for ODEs, N=150 for Simscape
Ts = T / N; % sampling time

sys_d = c2d(sys_c, Ts, 'zoh');

input_names = {'u1','u2','u3','d1','d2','d3'};
sys_d.InputName = input_names;
sys_d = setmpcsignals(sys_d, 'MV', 1:3, 'MD', 4:6);

% controller initialization
mpcobj = mpc(sys_d, Ts);

mpcobj.PredictionHorizon = 20;
mpcobj.ControlHorizon = 5;

mpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [0.01 0.01 0.01];
mpcobj.Weights.OutputVariables = [1 1 1 1 1 1];

% Optional constraints
mpcobj.MV(1).Min = -10; mpcobj.MV(1).Max = 10;
mpcobj.MV(2).Min = -10; mpcobj.MV(2).Max = 10;
mpcobj.MV(3).Min = -5;  mpcobj.MV(3).Max = 5;

assignin('base', 'mpcobj', mpcobj);