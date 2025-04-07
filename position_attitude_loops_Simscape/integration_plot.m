clc
%
% Run this file to plot after running the Simulink file
%

% Extract actual data and time
X_actual = timeseries(out.actual.X.Data, out.actual.X.Time);
Y_actual = timeseries(out.actual.Y.Data, out.actual.Y.Time);
Z_actual = timeseries(out.actual.Z.Data, out.actual.Z.Time);
Yaw_actual = timeseries(out.actual.yaw.Data * 180 / pi, out.actual.yaw.Time);
Pitch_actual = timeseries(out.actual.pitch.Data * 180 / pi, out.actual.pitch.Time);
Roll_actual = timeseries(out.actual.roll.Data * 180 / pi, out.actual.roll.Time);

% Extract reference data and time from out.ref_adm
X_ref = timeseries(out.ref.des_x.Data, out.ref.des_x.Time);
Y_ref = timeseries(out.ref.des_y.Data, out.ref.des_y.Time);
Z_ref = timeseries(out.ref.des_alt.Data, out.ref.des_alt.Time);
Yaw_ref = timeseries(out.ref.des_yaw.Data, out.ref.des_yaw.Time);
Pitch_ref = timeseries(out.ref.des_pitch.Data, out.ref.des_pitch.Time);
Roll_ref = timeseries(out.des_roll.Data, out.des_roll.Time);

% Create figure for trajectory
figure;

% X Position Plot
subplot(3,2,1);
plot(X_actual, 'b'); hold on; 
plot(X_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('X Position');
title('X Position: Actual vs Reference');
legend('Actual', 'Reference');

% Y Position Plot
subplot(3,2,2);
plot(Y_actual, 'b'); hold on; plot(Y_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Y Position');
title('Y Position: Actual vs Reference');
legend('Actual', 'Reference');

% Z Position Plot
subplot(3,2,3);
plot(Z_actual, 'b'); hold on; plot(Z_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Z Position');
title('Z Position: Actual vs Reference');
legend('Actual', 'Reference');

% Yaw Plot
subplot(3,2,4);
plot(Yaw_actual, 'b'); hold on; plot(Yaw_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Yaw (deg)');
title('Yaw: Actual vs Reference');
legend('Actual', 'Reference');

% Pitch Plot
subplot(3,2,5);
plot(Pitch_actual, 'b'); hold on; 
plot(Pitch_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Pitch: Actual vs Reference');
legend('Actual', 'Reference');

% Roll Plot
subplot(3,2,6);
plot(Roll_actual, 'b'); hold on; plot(Roll_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Roll (deg)');
title('Roll: Actual vs Reference');
legend('Actual', 'Reference');

% Adjust layout
sgtitle('Drone Actual vs Reference Data');

% Extract tilt and thrust data
tilt = out.Tilt.signals.values * 180 / pi;
t = out.Tilt.time;
tilt = reshape(tilt, size(tilt, 1), [])';
for i = 1 : length(tilt)
    if tilt(i) < 1e-3 && tilt(i) > -1e-3
        tilt(i) = 0;
    end
end
thrust = out.Thrust.signals.values;
thrust = reshape(thrust, size(thrust, 1), []);
tT = out.Thrust.time;

% Tilt angle
lineWidth = 1.4;
figure
plot(t, tilt, 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Tilt Angle Trajectory [deg]', 'FontSize', 12)
title('Tilt Angle vs. Time', 'FontSize', 14)
grid on

% Rotors thrust
figure
hold on
colors = lines(4);

for i = 1:4
    plot(tT, thrust(:,i), 'Color', colors(i,:), 'LineWidth', lineWidth)
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Rotors Thrust Values', 'FontSize', 12)
title('Rotors Thrust vs. Time', 'FontSize', 14)

legend({'thrust_1 [N]', 'thrust_2 [N]', 'thrust_3 [N]', 'thrust_4 [N]'}, ...
       'Location', 'best', 'FontSize', 10)

grid on
hold off

%% only for integration_model_5
% Extract rates
p = timeseries(out.roll_yaw_rates.p.Data, out.roll_yaw_rates.p.Time);
p_ref = timeseries(out.roll_yaw_rates.des_p.Data, out.roll_yaw_rates.des_p.Time);
r = timeseries(out.roll_yaw_rates.r.Data, out.roll_yaw_rates.r.Time);
r_ref = timeseries(out.roll_yaw_rates.des_r.Data, out.roll_yaw_rates.des_r.Time);
q = timeseries(out.pitch_rate.q.Data, out.pitch_rate.q.Time);
q_ref = timeseries(out.pitch_rate.des_q.Data, out.pitch_rate.des_q.Time);

% Create figure for rates 
figure

% Roll rate Plot
subplot(2,2,1);
plot(p, 'b'); hold on; 
plot(p_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Roll rate [rad/s]');
title('Roll rate: Actual vs Reference');
legend('Actual', 'Reference');

% Pitch rate Plot
subplot(2,2,2);
plot(q, 'b'); hold on; plot(q_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Pitch rate [rad/s]');
title('Pitch rate: Actual vs Reference');
legend('Actual', 'Reference');

% Yaw rate Plot
subplot(2,2,3);
plot(r, 'b'); hold on; plot(r_ref, 'r--'); hold off;
xlabel('Time (s)');
ylabel('Yaw rate [rad/s]');
title('Yaw rate: Actual vs Reference');
legend('Actual', 'Reference');

%% only for integration_model_5
% Extract pitch rate and torque
pitch_rate = out.Pitch_rate.signals.values;
t_p = out.Pitch_rate.time;
pitch_rate = reshape(pitch_rate, size(pitch_rate, 1), [])';
pitch_rate_des = out.Pitch_rate_des.signals.values;
t_p_des = out.Pitch_rate_des.time;
pitch_rate_des = reshape(pitch_rate_des, size(pitch_rate_des, 1), [])';
tau = out.Tau.signals.values;
t_tau = out.Tau.time;
tau_des = reshape(tau, size(tau, 1), [])';
tau = out.Tau1.signals.values;
t_tau1 = out.Tau1.time;
tau_pred = reshape(tau, size(tau, 1), [])';
tau = out.Tau2.signals.values;
t_tau2 = out.Tau2.time;
tau_pred2 = reshape(tau, size(tau, 1), [])';

% Pitch torque
lineWidth = 1.4;
figure
plot(t_tau, tau_des, 'LineWidth', lineWidth)
hold on
plot(t_tau2, tau_pred2,  'LineWidth', lineWidth)
hold on
plot(t_tau1, tau_pred,  'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Pitch Torque [N/m]', 'FontSize', 12)
title('Pitch Torque vs. Time', 'FontSize', 14)
legend('Wrt geometrical CoG', 'Wrt real CoG', 'Wrt real CoG & damped', 'Location', 'best', 'FontSize', 10)
grid on

% Pitch rate
lineWidth = 1.4;
figure
plot(t_p, pitch_rate, 'LineWidth', lineWidth)
hold on
plot(t_p_des, pitch_rate_des, 'LineWidth', lineWidth)
hold on 
plot(q_ref, 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Pitch Rate [rad/s]', 'FontSize', 12)
title('Pitch Rate: Actual vs Reference', 'FontSize', 14)
legend('Actual', 'Integrated reference','PID reference')
grid on

% % Pitch torque
% lineWidth = 1.4;
% figure
% plot(t_tau, tau_des, 'LineWidth', lineWidth)
% xlabel('Time (s)', 'FontSize', 12)
% ylabel('Pitch Torque [N/m]', 'FontSize', 12)
% title('Pitch Torque: PID control', 'FontSize', 14)
% grid on
