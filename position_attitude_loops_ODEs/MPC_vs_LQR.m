clc
clear
%
% Run this file to compare LQR and MPC controllers in ideal conditions
%

%% Optimal control
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
C = eye(6);
sys_c = ss(A, B, C, 0);

% discretization
T = 10; % time horizon
N = 40; % number of time steps (without t0)
Ts = T / N; % sampling time

sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

% matrix definition
n = size(Ad,1);
m = size(Bd,2);
G = zeros(n*(N+1), m*N);
H = zeros(n*(N+1), n);

for i = 2:N+1
    for j = 1:i-1
        k = i - j - 1;
        G((i-1)*n+1:i*n, (j-1)*m+1:j*m) = Ad^k * Bd;
    end
end

for i = 0:N
    H(i*n+1:(i+1)*n, :) = Ad^i;
end

% Initial condition
x0 = zeros(n,1);

% Desired reference state
xref = zeros(n,1);
xref(1) = 0.5;   % desired x position
xref(3) = 2;     % desired z position
xref(5) = 1;     % desired pitch

% Stack reference trajectory over horizon
r = repmat(xref, N+1, 1);

Q1_0 = 1;
Q2_0 = 0.1;
Qf = 10;

Q1 = diag(ones(n*(N+1),1)) * Q1_0;
for i = n*N + 1 : n*(N+1)
    Q1(i,i) = Qf;
end
Q2 = diag(ones(m*N,1)) * Q2_0;

P_lq = G'*Q1*G + Q2;
q_lq = (H*x0 - r)' * Q1 * G;

% optimization
U = -P_lq\q_lq';
U = reshape(U, m, N)';
U = [U; zeros(1,m)];

% simulate system
tspan = 0:Ts:T;
x0 = zeros(n,1);
[y, t, x_sol] = lsim(sys_d, U, tspan, x0);

lineWidth = 1.5;

figure
plot(t, x_sol(:,1), t, x_sol(:,3), t, x_sol(:,5), 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Trajectory Values', 'FontSize', 12)
title('LQR STate Trajectories vs. Time', 'FontSize', 14)
legend("x [m]", "z [m]", "pitch [rad]", 'FontSize', 10)
grid on

figure
stairs(t, U(:,1), 'LineWidth', lineWidth);
hold on
stairs(t, U(:,2), 'LineWidth', lineWidth);
hold on
stairs(t, U(:,3), 'LineWidth', lineWidth);
xlabel('Time (s)', 'FontSize', 12)
ylabel('Control Input', 'FontSize', 12)
legend('u_x', 'u_z', 'u_{\theta}', 'FontSize', 10)
title('LQR Control Inputs', 'FontSize', 14)
grid on

%% MPC
% Model parameters
mass = 1.12;
g = 9.81;
I_y = 0.035;

% Continuous-time system
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
C = eye(6);
sys_c = ss(A, B, C, 0);

% Discretization
Ts = 0.1;          % sampling time
N = 20;            % prediction horizon (shorter in MPC for speed)
T_total = 10;      % total simulation time
steps = T_total / Ts;

sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

% Sizes
n = size(Ad,1);    % number of states
m = size(Bd,2);    % number of inputs

% Cost weights
Q1_0 = 1;
Qf = 10;
Q2_0 = 0.1;

Q1 = diag(ones(n*(N+1),1)) * Q1_0;
for i = n*N + 1 : n*(N+1)
    Q1(i,i) = Qf;
end
Q2 = diag(ones(m*N,1)) * Q2_0;

% Reference state
xref = [0.5; 0; 2; 0; 1; 0];  % desire reference trajectory (x,vx,z,vz,theta,q)
r = repmat(xref, N+1, 1);   % stacked reference trajectory

% Preallocation
x0 = zeros(n,1);
x = x0;
X_log = zeros(steps+1, n);
U_log = zeros(steps, m);
X_log(1,:) = x0';

for k = 1:steps
    % Build G and H
    G = zeros(n*(N+1), m*N);
    H = zeros(n*(N+1), n);
    for i = 2:N+1
        for j = 1:i-1
            k_ij = i - j - 1;
            G((i-1)*n+1:i*n, (j-1)*m+1:j*m) = Ad^k_ij * Bd;
        end
    end
    for i = 0:N
        H(i*n+1:(i+1)*n, :) = Ad^i;
    end

    % Compute cost function
    P_lq = G' * Q1 * G + Q2;
    q_lq = (H*x - r)' * Q1 * G;

    % Solve (no constraints)
    U_opt = -P_lq \ q_lq';
    U_opt = reshape(U_opt, m, N)';
    u_apply = U_opt(1,:)';

    % Apply control input and update state
    x = Ad * x + Bd * u_apply;

    % Log
    X_log(k+1,:) = x';
    U_log(k,:) = u_apply';
end

% Time vector for plotting
t = 0:Ts:T_total;

% Plot trajectory
figure
plot(t, X_log(:,1), t, X_log(:,3), t, X_log(:,5), 'LineWidth', 1.5)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Trajectory Values', 'FontSize', 12)
title('MPC State Trajectories', 'FontSize', 14)
legend("x [m]", "z [m]", "pitch [rad]", 'FontSize', 10)
grid on

% Optional: plot control inputs
figure
stairs(0:Ts:T_total-Ts, U_log, 'LineWidth', 1.5)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Control Input', 'FontSize', 12)
legend('u_x', 'u_z', 'u_{\theta}', 'FontSize', 10)
title('MPC Control Inputs', 'FontSize', 14)
grid on

