% =========================================================
% uav_trajectory.m — UAV Flight Path Definitions
%
% PURPOSE:
%   Define two different UAV flight paths:uav_trajectory.m
%     1. Straight line  — constant velocity, no turns
%     2. Zig-zag         — sinusoidal lateral oscillation
%
%   This is the ONLY place where "UAV path" is defined.
%   Both camera_projection.m and the observability analysis
%   take UAV position (uav_x, uav_y) as a given input —
%   they do not care HOW it was computed.
%
%   This file just demonstrates both paths and plots them.
%   The actual functions are copied inline into
%   main_compare.m so that file stays self-contained.
%
% HOW TO RUN:
%   Press F5 or type: uav_trajectory
%   (Self-contained — no other files required)
% =========================================================

clear; clc; close all;

fprintf('=========================================\n');
fprintf('  UAV TRAJECTORY — Straight vs Zig-Zag\n');
fprintf('=========================================\n\n');

%% ---- Parameters ----
dt      = 0.5;
T_total = 60;
t       = 0:dt:T_total;
N_steps = length(t);

UAV_x0 = 0;     % Start X [m]
UAV_y0 = 0;     % Start Y [m]
UAV_vx = 0.5;   % Forward velocity [m/s] — same for both paths

%% ====================================================
%%  PATH 1: STRAIGHT LINE  (same as your existing code)
%% ====================================================
fprintf('--- PATH 1: Straight Line ---\n\n');

% This is exactly what camera_projection.m currently does:
%   uav_x = UAV_x0 + UAV_vx*(k-1)*dt
%   uav_y = UAV_y0 + UAV_vy*(k-1)*dt
% with UAV_vy a small constant (0.2 m/s) — a gentle straight diagonal.

UAV_vy_straight = 0.2;   % Constant lateral velocity [m/s]

uav_x_straight = zeros(1, N_steps);
uav_y_straight = zeros(1, N_steps);

for k = 1:N_steps
    uav_x_straight(k) = UAV_x0 + UAV_vx * (k-1) * dt;
    uav_y_straight(k) = UAV_y0 + UAV_vy_straight * (k-1) * dt;
end

fprintf('Straight line: y = UAV_y0 + UAV_vy*t\n');
fprintf('  Start: (%.1f, %.1f)   End: (%.1f, %.1f)\n\n', ...
    uav_x_straight(1), uav_y_straight(1), ...
    uav_x_straight(end), uav_y_straight(end));

%% ====================================================
%%  PATH 2: ZIG-ZAG  (new)
%% ====================================================
fprintf('--- PATH 2: Zig-Zag ---\n\n');

% Forward motion (x) stays the same — UAV still progresses forward.
% Lateral motion (y) now oscillates sinusoidally instead of drifting
% at constant velocity. This introduces extra relative motion between
% UAV and ship, which is the whole point of the comparison.

amplitude = 8;        % Zig-zag swing amplitude [m]
frequency = 0.15;     % Oscillation frequency [rad/s]

uav_x_zigzag = zeros(1, N_steps);
uav_y_zigzag = zeros(1, N_steps);

for k = 1:N_steps
    uav_x_zigzag(k) = UAV_x0 + UAV_vx * (k-1) * dt;
    uav_y_zigzag(k) = UAV_y0 + amplitude * sin(frequency * (k-1) * dt);
end

fprintf('Zig-zag: y = UAV_y0 + amplitude*sin(frequency*t)\n');
fprintf('  amplitude = %.0f m,  frequency = %.2f rad/s\n', amplitude, frequency);
fprintf('  Start: (%.1f, %.1f)   End: (%.1f, %.1f)\n\n', ...
    uav_x_zigzag(1), uav_y_zigzag(1), ...
    uav_x_zigzag(end), uav_y_zigzag(end));

%% ====================================================
%%  COMPARE LATERAL VELOCITY (this is what differs)
%% ====================================================
fprintf('--- Comparing lateral velocity profiles ---\n\n');

% Straight line: vy is constant
vy_straight = UAV_vy_straight * ones(1, N_steps);

% Zig-zag: vy = d/dt[amplitude*sin(freq*t)] = amplitude*freq*cos(freq*t)
vy_zigzag = amplitude * frequency * cos(frequency * t);

fprintf('Straight line vy: constant at %.2f m/s\n', UAV_vy_straight);
fprintf('Zig-zag vy: oscillates between %.2f and %.2f m/s\n\n', ...
    min(vy_zigzag), max(vy_zigzag));

fprintf('This oscillating velocity is the "extra excitation" that\n');
fprintf('classical observability theory says should improve the\n');
fprintf('weakest eigenvalue of the Gramian, compared to a UAV moving\n');
fprintf('at constant relative velocity to the ship.\n\n');

%% ====================================================
%%  PLOTS
%% ====================================================

c_straight = [0.15 0.45 0.80];
c_zigzag   = [0.85 0.33 0.10];

% ---- Fig 1: Both flight paths (X-Y) ----
figure(1); set(gcf,'Name','Traj Fig 1 - UAV Flight Paths');
plot(uav_x_straight, uav_y_straight, '-', 'Color',c_straight,'LineWidth',2); hold on;
plot(uav_x_zigzag,   uav_y_zigzag,   '-', 'Color',c_zigzag,  'LineWidth',2);
plot(uav_x_straight(1), uav_y_straight(1), 'ko', 'MarkerSize',8,'MarkerFaceColor','k');
xlabel('X [m]'); ylabel('Y [m]');
title('Traj Fig 1 - UAV Flight Paths: Straight vs Zig-Zag');
legend('Straight line','Zig-zag','Start','Location','best');
grid on; axis equal;

% ---- Fig 2: Y position over time ----
figure(2); set(gcf,'Name','Traj Fig 2 - Lateral Position vs Time');
plot(t, uav_y_straight, '-', 'Color',c_straight,'LineWidth',2); hold on;
plot(t, uav_y_zigzag,   '-', 'Color',c_zigzag,  'LineWidth',2);
xlabel('Time [s]'); ylabel('UAV Y position [m]');
title('Traj Fig 2 - Lateral Position vs Time');
legend('Straight line','Zig-zag','Location','best'); grid on;

% ---- Fig 3: Lateral velocity over time ----
figure(3); set(gcf,'Name','Traj Fig 3 - Lateral Velocity vs Time');
plot(t, vy_straight, '-', 'Color',c_straight,'LineWidth',2); hold on;
plot(t, vy_zigzag,   '-', 'Color',c_zigzag,  'LineWidth',2);
xlabel('Time [s]'); ylabel('UAV lateral velocity vy [m/s]');
title('Traj Fig 3 - Lateral Velocity vs Time (the key difference)');
legend('Straight line (constant)','Zig-zag (oscillating)','Location','best');
grid on;

fprintf('uav_trajectory.m complete. 3 figures generated.\n');