clc
%
% Run this file to plot after running the Simulink file
%

% extract outputs
traj = out.Trajectory.signals.values;
traj = reshape(traj, size(traj, 1), [])';
vel = out.Velocity.signals.values;
vel = reshape(vel, size(vel,1), [])';
vel_ref = out.Velocity_ref.signals.values;
vel_ref = reshape(vel_ref, size(vel_ref,1), []);
t = out.tout;
ref = out.Reference.signals.values;
ref_u = out.Reference_u.signals.values;
if size(ref_u,1) == 1
    ref_u = ref_u*ones(length(t),1);
end
if size(ref,1) == 1
    ref_new = zeros(size(t,1),6);
    for i = 1 : size(t,1)
        ref_new(i,:) = ref;
    end
    ref = ref_new;
end
ref = [ref(:,1:5) ref_u];
tilt = out.Tilt.signals.values;
tilt = reshape(tilt, size(tilt, 1), [])';
for i = 1 : length(tilt)
    if tilt(i) < 1e-3 && tilt(i) > -1e-3
        tilt(i) = 0;
    end
end
thrust = out.Thrust.signals.values;
thrust = reshape(thrust, size(thrust, 1), []);

% plot trajectory (x, y, z, yaw, pitch, roll)
figure
hold on
colors = lines(6);
lineWidth = 1.5;
dashLineWidth = 1.2;

for i = 1:6
    plot(t, traj(:,i), 'Color', colors(i,:), 'LineWidth', lineWidth)
end

for i = 1:6
        plot(t, ref(:,i), '--', 'Color', colors(i,:), 'LineWidth', dashLineWidth)
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Trajectory Values', 'FontSize', 12)
title('Trajectory Components vs. Time with Reference Targets', 'FontSize', 14)

legend({'x [m]', 'y [m]', 'z [m]', 'yaw [rad]', 'pitch [rad]', 'roll [rad]'}, ...
       'Location', 'best', 'FontSize', 10)

grid on
hold off

% plot tilt angle
figure
plot(t, tilt, 'LineWidth', lineWidth)
xlabel('Time (s)', 'FontSize', 12)
ylabel('Tilt Angle Trajectory [deg]', 'FontSize', 12)
title('Tilt Angle vs. Time', 'FontSize', 14)
grid on

% plot rotors thrust
figure
hold on
colors = lines(4);
lineWidth = 1.5;

for i = 1:4
    plot(t, thrust(:,i), 'Color', colors(i,:), 'LineWidth', lineWidth)
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Rotors Thrust Values', 'FontSize', 12)
title('Rotors Thrust vs. Time', 'FontSize', 14)

legend({'thrust_1 [N]', 'thrust_2 [N]', 'thrust_3 [N]', 'thrust_4 [N]'}, ...
       'Location', 'best', 'FontSize', 10)

grid on
hold off

% plot velocities (vx, vy, vz) 
% local frame for model_5 and model_6, global_frame for model_3 and model_4
figure
hold on
colors = lines(3);
lineWidth = 1.5;
dashLineWidth = 1.2;

for i = 1:3
    plot(t, vel(:,i), 'Color', colors(i,:), 'LineWidth', lineWidth)
end

for i = 1:3
        plot(t, vel_ref(:,i), '--', 'Color', colors(i,:), 'LineWidth', dashLineWidth)
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Velocity values', 'FontSize', 12)
title('Velocity Components vs. Time with Reference Targets', 'FontSize', 14)

legend({'vx [m]', 'vy [m]', 'vz [m]'}, ...
       'Location', 'best', 'FontSize', 10)

grid on
hold off

% % plot x-y trajectory
% figure
% hold on
% lineWidth = 1.5;
% dashLineWidth = 1.2;
% 
% plot(traj(:,1), traj(:,2), 'LineWidth', lineWidth)
% hold on
% plot(ref(:,1), ref(:,2), '*', 'LineWidth', dashLineWidth)
% 
% xlabel('x', 'FontSize', 12)
% ylabel('y', 'FontSize', 12)
% title('Trajectory', 'FontSize', 14)
% 
% grid on
% hold off

% % plot pitch torque (for model 5)
% tau = out.Tau.signals.values;
% tau = reshape(tau, size(tau, 1), []);
% figure
% plot(t, tau, 'LineWidth', lineWidth)
% xlabel('Time (s)', 'FontSize', 12)
% ylabel('Pitch Torque [Nm]', 'FontSize', 12)
% title('Pitch Torque vs. Time', 'FontSize', 14)
% grid on



