%% Inverted Pendulum - Combined Plant + Controller Visualizer
% Overlays true plant states with controller estimates and internals
% Two CSVs at different sample rates, aligned by timestamp
clear; clc; close all;

%% ========== LOAD DATA ==========

plant_csv_path = "results\normal\plant_state.csv";
ctrl_csv_path  = "results\normal\control.csv";
imu_csv_path = "results\normal\imu.csv";
enc_csv_path = "results\normal\encoder.csv";

P = readtable(plant_csv_path);
C = readtable(ctrl_csv_path);
IMU = readtable(imu_csv_path);
ENC = readtable(enc_csv_path);

% Plant signals
pt          = P.time_s;
p_theta     = P.theta;
p_theta_d   = P.theta_dot;
p_theta_dd  = P.theta_ddot;
p_z         = P.z;
p_z_d       = P.z_dot;
p_z_dd      = P.z_ddot;
p_F         = P.force_applied;
p_tau       = P.tau_disturbance;

% Controller signals
ct                = C.time_s;
c_theta_est_accel = C.theta_est_accel;
c_theta_est       = C.theta_est;
c_theta_d_est     = C.theta_dot_est;
c_theta_dd_est    = C.theta_ddot_est;
c_z_est           = C.z_est;
c_z_d_est         = C.z_dot_est;
c_z_dd_est        = C.z_ddot_est;
c_outer_P         = C.outer_P;
c_outer_I         = C.outer_I;
c_outer_D         = C.outer_D;
c_sp_raw          = C.theta_setpoint_raw;
c_sp              = C.theta_setpoint;
c_e_theta         = C.e_theta;
c_inner_P         = C.inner_P;
c_inner_I         = C.inner_I;
c_inner_D         = C.inner_D;
c_F_raw           = C.force_raw;
c_F               = C.force;

% IMU signals
it        = IMU.time_s;
i_omega   = IMU.omega;
i_ax      = IMU.a_x_prime;
i_ay      = IMU.a_y_prime;
i_dist    = IMU.disturbed;

% Encoder signals
et        = ENC.time_s;
e_z       = ENC.z_quantized;

% Pendulum length — must match simulation
L = 0.5;

%% ========== FIGURE 1: STATE COMPARISONS ==========

figure('Name','States: True vs Estimated','Position',[50 50 1300 900]);

% --- Theta ---
subplot(3,2,1);
plot(pt, rad2deg(mod(p_theta+pi,2*pi)-pi), 'b', 'LineWidth', 1.2); hold on;
plot(ct, rad2deg(c_theta_est), 'r--', 'LineWidth', 1.0);
plot(ct, rad2deg(c_theta_est_accel), 'g-.', 'LineWidth', 0.8);
plot(ct, rad2deg(c_sp), 'Color', [0.5 0.5 0], 'LineWidth', 0.8);
plot(ct, rad2deg(c_sp_raw), ':', 'Color', [0.5 0.5 0], 'LineWidth', 0.8);
yline(0, 'k--', 'LineWidth', 0.5);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('\theta: True vs Estimated vs Setpoint');
legend('True', 'Estimated', 'Estimated (accel only)', 'Setpoint (clamped)', 'Setpoint (raw)', ...
       'Location', 'best', 'FontSize', 7);
grid on;

% --- Theta dot ---
subplot(3,2,2);
plot(pt, rad2deg(p_theta_d), 'b', 'LineWidth', 1.2); hold on;
plot(ct, rad2deg(c_theta_d_est), 'r--', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Angular Velocity [deg/s]');
title('$$\dot{\theta}$$: True vs Estimated', 'Interpreter', 'latex');
legend('True', 'Estimated', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Z ---
subplot(3,2,3);
plot(pt, p_z, 'b', 'LineWidth', 1.2); hold on;
plot(ct, c_z_est, 'r--', 'LineWidth', 1.0);
yline(0, 'k--', 'LineWidth', 0.5);
xlabel('Time [s]'); ylabel('Position [m]');
title('z: True vs Estimated');
legend('True', 'Estimated', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Z dot ---
subplot(3,2,4);
plot(pt, p_z_d, 'b', 'LineWidth', 1.2); hold on;
plot(ct, c_z_d_est, 'r--', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('$$\dot{z}$$: True vs Estimated', 'Interpreter', 'latex');
legend('True', 'Estimated', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Theta ddot ---
subplot(3,2,5);
plot(pt, p_theta_dd, 'b', 'LineWidth', 1.2); hold on;
plot(ct, c_theta_dd_est, 'r--', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Angular Accel [rad/s^2]');
title('$$\ddot{\theta}$$: True vs Estimated', 'Interpreter', 'latex');
legend('True', 'Estimated', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Z ddot ---
subplot(3,2,6);
plot(pt, p_z_dd, 'b', 'LineWidth', 1.2); hold on;
plot(ct, c_z_dd_est, 'r--', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Accel [m/s^2]');
title('$$\ddot{z}$$: True vs Estimated', 'Interpreter', 'latex');
legend('True', 'Estimated', 'Location', 'best', 'FontSize', 7);
grid on;

sgtitle('True Plant States vs Controller Estimates', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 2: ESTIMATION ERRORS ==========

% Interpolate plant onto controller timestamps for error computation
p_theta_at_ct    = interp1(pt, p_theta,    ct, 'linear', NaN);
p_theta_d_at_ct  = interp1(pt, p_theta_d,  ct, 'linear', NaN);
p_theta_dd_at_ct = interp1(pt, p_theta_dd, ct, 'linear', NaN);
p_z_at_ct        = interp1(pt, p_z,        ct, 'linear', NaN);
p_z_d_at_ct      = interp1(pt, p_z_d,      ct, 'linear', NaN);
p_z_dd_at_ct     = interp1(pt, p_z_dd,     ct, 'linear', NaN);

figure('Name','Estimation Errors','Position',[80 80 1200 900]);

subplot(3,2,1);
plot(ct, rad2deg(p_theta_at_ct - c_theta_est), 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [deg]');
title('\theta Estimation Error (true - est)');
grid on;

subplot(3,2,2);
plot(ct, rad2deg(p_theta_d_at_ct - c_theta_d_est), 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [deg/s]');
title('$$\dot{\theta}$$ Estimation Error', 'Interpreter', 'latex');
grid on;

subplot(3,2,3);
plot(ct, p_z_at_ct - c_z_est, 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [m]');
title('z Estimation Error');
grid on;

subplot(3,2,4);
plot(ct, p_z_d_at_ct - c_z_d_est, 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [m/s]');
title('$$\dot{z}$$ Estimation Error', 'Interpreter', 'latex');
grid on;

subplot(3,2,5);
plot(ct, p_theta_dd_at_ct - c_theta_dd_est, 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [rad/s^2]');
title('$$\ddot{\theta}$$ Estimation Error', 'Interpreter', 'latex');
grid on;

subplot(3,2,6);
plot(ct, p_z_dd_at_ct - c_z_dd_est, 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [m/s^2]');
title('$$\ddot{z}$$ Estimation Error', 'Interpreter', 'latex');
grid on;

sgtitle('Sensor Fusion Estimation Errors', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 3: CONTROLLER INTERNALS ==========

figure('Name','Controller Internals','Position',[100 60 1300 900]);

% --- Outer loop PID contributions ---
subplot(3,2,1);
plot(ct, c_outer_P, 'b', 'LineWidth', 1.0); hold on;
plot(ct, c_outer_I, 'Color', [0 0.6 0], 'LineWidth', 1.0);
plot(ct, c_outer_D, 'r', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('[rad]');
title('Outer Loop (z \rightarrow \theta_{sp}): PID Terms');
legend('P', 'I', 'D', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Theta setpoint raw vs clamped ---
subplot(3,2,2);
plot(ct, rad2deg(c_sp_raw), 'Color', [0.7 0.5 0], 'LineWidth', 1.0); hold on;
plot(ct, rad2deg(c_sp), 'Color', [0 0 0.8], 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('\theta Setpoint: Raw vs Clamped');
legend('Raw', 'Clamped', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Inner loop error ---
subplot(3,2,3);
plot(ct, rad2deg(c_e_theta), 'b', 'LineWidth', 1.0);
yline(0, 'k--', 'LineWidth', 0.5);
xlabel('Time [s]'); ylabel('Error [deg]');
title('Inner Loop: \theta Error (setpoint - est)');
grid on;

% --- Inner loop PID contributions ---
subplot(3,2,4);
plot(ct, c_inner_P, 'b', 'LineWidth', 1.0); hold on;
plot(ct, c_inner_I, 'Color', [0 0.6 0], 'LineWidth', 1.0);
plot(ct, c_inner_D, 'r', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('[N]');
title('Inner Loop (\theta \rightarrow F): PID Terms');
legend('P', 'I', 'D', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Force raw vs saturated ---
subplot(3,2,5);
plot(ct, c_F_raw, 'Color', [0.7 0 0], 'LineWidth', 1.0); hold on;
plot(ct, c_F, 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Force [N]');
title('Force: Raw vs Saturated');
legend('Raw (unbounded)', 'Applied (saturated)', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Disturbance torque ---
subplot(3,2,6);
stem(pt, p_tau, 'Color', [0.8 0 0.4], 'Marker', 'none', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Torque [N\cdotm]');
title('Disturbance Torque');
grid on;

sgtitle('Controller Internals: Cascade PID', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 4: PHASE PORTRAITS ==========

figure('Name','Phase Portraits','Position',[120 80 1000 450]);

subplot(1,2,1);
plot(rad2deg(p_theta), rad2deg(p_theta_d), 'b', 'LineWidth', 0.8); hold on;
plot(rad2deg(p_theta(1)), rad2deg(p_theta_d(1)), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(rad2deg(p_theta(end)), rad2deg(p_theta_d(end)), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Angle [deg]'); ylabel('Angular Velocity [deg/s]');
title('\theta Phase Portrait');
legend('Trajectory', 'Start', 'End', 'Location', 'best');
grid on;

subplot(1,2,2);
plot(p_z, p_z_d, 'Color', [0 0.6 0], 'LineWidth', 0.8); hold on;
plot(p_z(1), p_z_d(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(p_z(end), p_z_d(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Position [m]'); ylabel('Velocity [m/s]');
title('Cart Phase Portrait');
legend('Trajectory', 'Start', 'End', 'Location', 'best');
grid on;

sgtitle('Phase Portraits', 'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 5: SENSOR DATA ==========

% Compute true (noiseless) IMU values from plant for comparison
% Interpolate plant onto IMU timestamps
ip_theta    = interp1(pt, p_theta,    it, 'linear', NaN);
ip_theta_d  = interp1(pt, p_theta_d,  it, 'linear', NaN);
ip_theta_dd = interp1(pt, p_theta_dd, it, 'linear', NaN);
ip_z_dd     = interp1(pt, p_z_dd,     it, 'linear', NaN);

% True accelerometer values (same equations as IMUSensor.cpp)
% a_x' = -L*theta_ddot + g*sin(theta) + z_ddot*cos(theta)
% a_y' = -L*theta_dot^2 + g*cos(theta) - z_ddot*sin(theta)
true_ax = -L.*ip_theta_dd + 9.81.*sin(ip_theta) + ip_z_dd.*cos(ip_theta);
true_ay = -L.*ip_theta_d.^2 + 9.81.*cos(ip_theta) - ip_z_dd.*sin(ip_theta);

true_ax_g = 9.81.*sin(ip_theta);
true_ay_g = 9.81.*cos(ip_theta);

figure('Name','Sensor Data','Position',[130 40 1300 900]);

% --- Gyro: omega vs true theta_dot ---
hold off
subplot(3,2,1);
plot(it, i_omega, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 2); hold on;
plot(pt, p_theta_d, 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('[rad/s]');
title('Gyro: \omega (measured) vs true $$\dot{\theta}$$', 'Interpreter', 'latex');
legend('Gyro (noisy)', 'True \theta''', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Gyro noise residual ---
subplot(3,2,2);
plot(it, i_omega - ip_theta_d, '.', 'Color', [0.8 0 0], 'MarkerSize', 2);
xlabel('Time [s]'); ylabel('[rad/s]');
title('Gyro Noise (measured - true)');
grid on;

% --- Accelerometer x' ---
subplot(3,2,3);
plot(it, i_ax, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 2); hold on;
plot(it, true_ax, 'b', 'LineWidth', 1.0);
plot(it, true_ax_g, 'r-', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('[m/s^2]');
title('Accel a_{x''}: Measured vs True');
legend('Measured', 'True', 'Gravity Only', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Accelerometer y' ---
subplot(3,2,4);
plot(it, i_ay, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 2); hold on;
plot(it, true_ay, 'b', 'LineWidth', 1.0);
plot(it, true_ay_g, 'r-', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('[m/s^2]');
title('Accel a_{y''}: Measured vs True');
legend('Measured', 'True', 'Gravity Only', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Encoder: z_quantized vs true z ---
subplot(3,2,5);
stairs(et, e_z, 'Color', [0 0.5 0], 'LineWidth', 1.0); hold on;
plot(pt, p_z, 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('[m]');
title('Encoder: z_{quantized} vs True z');
legend('Encoder (quantized)', 'True z', 'Location', 'best', 'FontSize', 7);
grid on;

% --- Encoder quantization error ---
subplot(3,2,6);
ep_z = interp1(pt, p_z, et, 'linear', NaN);
stairs(et, e_z - ep_z, 'Color', [0.8 0 0], 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('[m]');
title('Encoder Quantization Error');
grid on;

sgtitle('Sensor Data: IMU + Encoder', 'FontSize', 14, 'FontWeight', 'bold');




%% ========== ANIMATION ==========

figure('Name','Pendulum Animation','Position',[150 100 900 550]);

% Use plant timestamps for animation (higher rate)
N = length(pt);
target_fps = 30;
dt_avg = mean(diff(pt));
skip = max(1, round(1 / (target_fps * dt_avg)));
idx = 1:skip:N;

% Interpolate controller setpoint onto plant timestamps for animation
c_sp_at_pt = interp1(ct, c_sp, pt, 'previous', NaN);
c_F_at_pt  = interp1(ct, c_F,  pt, 'previous', NaN);

cart_w = 0.15;
cart_h = 0.08;

for k = idx
    clf;

    z_k   = p_z(k);
    th_k  = p_theta(k);
    sp_k  = c_sp_at_pt(k);
    F_k   = c_F_at_pt(k);
    tau_k = p_tau(k);

    % True pendulum tip (CCW-positive)
    pend_x = z_k - L*sin(th_k);
    pend_y = L*cos(th_k);

    % Ghost pendulum at setpoint angle
    ghost_x = z_k - L*sin(sp_k);
    ghost_y = L*cos(sp_k);

    % Rail
    rail_span = 1.0;
    plot([z_k - rail_span, z_k + rail_span], [0 0], 'k-', 'LineWidth', 2);
    hold on;

    % Cart
    rectangle('Position', [z_k - cart_w, -cart_h/2, 2*cart_w, cart_h], ...
              'Curvature', 0.2, 'FaceColor', [0.3 0.5 0.8], ...
              'EdgeColor', 'k', 'LineWidth', 1.5);

    % Wheels
    wheel_r = 0.02;
    circ = linspace(0, 2*pi, 30);
    for wz = [z_k - cart_w*0.6, z_k + cart_w*0.6]
        fill(wz + wheel_r*cos(circ), ...
             -cart_h/2 + wheel_r*sin(circ) - wheel_r, [0.3 0.3 0.3]);
    end

    % Ghost pendulum (setpoint) — draw first so real one is on top
    if ~isnan(sp_k)
        plot([z_k, ghost_x], [0, ghost_y], '--', ...
             'Color', [0.4 0.8 0.4 0.6], 'LineWidth', 2);
        plot(ghost_x, ghost_y, 'o', 'MarkerSize', 10, ...
             'MarkerFaceColor', [0.4 0.8 0.4], 'MarkerEdgeColor', 'none');
    end

    % True pendulum rod
    plot([z_k, pend_x], [0, pend_y], 'Color', [0.7 0.2 0.2], 'LineWidth', 3);

    % Tip mass
    plot(pend_x, pend_y, 'o', 'MarkerSize', 12, ...
         'MarkerFaceColor', [0.8 0.1 0.1], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

    % Pivot
    plot(z_k, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

    % Force arrow
    if ~isnan(F_k) && abs(F_k) > 0.5
        quiver(z_k, -cart_h, F_k*0.01, 0, 0, ...
               'MaxHeadSize', 0.8, 'LineWidth', 2, 'Color', [0 0.7 0]);
    end

    % Disturbance indicator
    if abs(tau_k) > 0.01
        text(z_k, L + 0.08, sprintf('\\tau_d = %.1f', tau_k), ...
             'HorizontalAlignment', 'center', 'Color', [0.8 0 0.4], ...
             'FontSize', 11, 'FontWeight', 'bold');
    end

    % Title
    sp_deg = '---';
    if ~isnan(sp_k)
        sp_deg = sprintf('%.2f', rad2deg(sp_k));
    end
    title(sprintf('t=%.3fs | \\theta=%.2f° | \\theta_{sp}=%s° | z=%.4fm | F=%.1fN', ...
          pt(k), rad2deg(th_k), sp_deg, z_k, F_k), 'FontSize', 11);

    xlim([z_k - 0.8, z_k + 0.8]);
    ylim([-0.3, 0.7]);
    axis equal;
    grid on;

    drawnow;
    pause(dt_avg * skip * 0.5);
end