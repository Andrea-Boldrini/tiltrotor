clc
%
% Run this file to generate optimal pitching controller
%

% model parameters
m = 1.12;
g = 9.81;
I_y = 0.035; %I_x = 0.018 and I_z = 0.051
%m = 5.1664;
%g = 9.8066;
%I_y = 0.15;

% linearized system
A = [0 1 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0];
B = [0 0 0;
    1/m 0 0;
    0 0 0;
    0 1/m 0;
    0 0 0
    0 0 1/I_y];
C = 5*eye(6); % maximum multiplicator is 25
C(6,6) = C(6,6);

% observability and controllability analysis
Co = ctrb(A, B); % to be checked to be full ranked
if rank(Co) == 6
    disp('System controllable') 
else
    disp('System not controllable')
end
Ob = obsv(A, C); % to be checked to be full ranked
if rank(Ob) == 6
    disp('System observable') 
else
    disp('System not observable')
end

% optimization matrices
Q = C'*C; % to be checked to be positive definite
det_Q = det(Q);
if det_Q > 0
    disp('Q is positive definite');
else
    disp('Q is not positive definite')
end
R = [1 0 0;
    0 1 0;
    0 0 1]; % to be checked to be positive definite
det_R = det(R);
if det_R > 0
    disp('R is positive definite');
else
    disp('R is not positive definite')
end
R_inv = inv(R);

% Riccati equation
[P_solution, K, L] = icare(A, B, Q, R);
[X] = Schur_solver(A, B, Q, R);

% Display the numerical solution
disp('Numerical solution for P:');
disp(P_solution);
disp('Determinant of P:');
disp(det(P_solution));
if det(P_solution) > 0
    disp('P positive definite')
end

disp('Numerical solution for X:');
disp(X);
disp('Determinant of X:');
disp(det(X));
if det(X) > 0
    disp('P positive definite')
end

%% Tests on designed optimal controller
clc
% x = [x vx z vz pitch q]
x0 = [0 0 0 0 0 0]';
xf = [0 0 0 0 0*pi/180 0]';
tspan = [0 20];
[t, x_sol] = ode45(@(t, x) system_ode(t, x, A, B, K, xf), tspan, x0);

lineWidth = 1.5;

figure
plot(t, x_sol(:,1), t, x_sol(:,3), t, x_sol(:,5), 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Trajectory Values', 'FontSize', 12)
title('Trajectory Components vs. Time', 'FontSize', 14)
legend("x [m]", "z [m]", "pitch [rad]", 'FontSize', 10)
grid on

figure
plot(t, x_sol(:,2), t, x_sol(:,4), t, x_sol(:,6), 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Velocity Values', 'FontSize', 12)
title('Velocity Components vs. Time', 'FontSize', 14)
legend("velocity x", "velocity z", "rate of pitch", 'FontSize', 10)
grid on

u = zeros(length(t), 3);
tilt = zeros(length(t), 1);
for i = 1 : length(t)
     u(i,:) = (-K*(x_sol(i,:)'-xf))';
     u(i,1) = u(i,1) - m*g*sin(x_sol(i,5));
     u(i,2) = u(i,2) + m*g*cos(x_sol(i,5));
     tilt(i) = atan2(u(i,1),u(i,2))*180/pi;
     if tilt(i) < 0.01 && tilt(i) > -0.01
         tilt(i) = 0;
     end
end

figure
plot(t, u(:,1), t, u(:,2), t, u(:,3), 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Input', 'FontSize', 12)
title('Input Components vs. Time', 'FontSize', 14)
legend("fx", "fz", "tau", 'FontSize', 10)
grid on

figure
plot(t, tilt, 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Tilt angle', 'FontSize', 12)
title('Tilt Angle vs. Time', 'FontSize', 14)
legend("tilt [deg]", 'FontSize', 10)
grid on

function dSdt = system_ode(t, x, A, B, K, xf)
    dSdt = (A-B*K)*(x-xf);
end
