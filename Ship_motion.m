% =========================================================
% ship_motion.m — Ship Trajectory Generator
%
% PURPOSE:
%   Show how the ship moves over time using two motion models:
%     - CV (Constant Velocity):     state = [x, y, vx, vy]
%     - CA (Constant Acceleration): state = [x, y, vx, vy, ax, ay]
%
%   The core equation is:
%       X(k+1) = A * X(k)
%
%   A is the state transition matrix.
%   We apply it repeatedly to get the full trajectory.
%
% HOW TO RUN:
%   Press F5 or type: ship_motion
%   (No inputs needed — everything is defined inside)
% =========================================================

clear; clc; close all;

fprintf('=========================================\n');
fprintf('  SHIP MOTION — State Propagation\n');
fprintf('=========================================\n\n');

%% ---- Parameters (self-contained, no config needed) ----
dt      = 0.5;
T_total = 60;
t       = 0:dt:T_total;
N_steps = length(t);

%% ====================================================
%%  PART 1: CONSTANT VELOCITY (CV) MODEL
%% ====================================================
fprintf('--- PART 1: Constant Velocity (CV) Model ---\n\n');

% State vector: X = [x, y, vx, vy]
% ----------------------------------
% x(k+1)  = x(k)  + vx(k)*dt     <- position updates with velocity
% y(k+1)  = y(k)  + vy(k)*dt
% vx(k+1) = vx(k)                 <- velocity stays constant
% vy(k+1) = vy(k)

% Written as a matrix:
%
%   [x ]       [1  0  dt  0 ] [x ]
%   [y ]     = [0  1  0   dt] [y ]
%   [vx]  k+1  [0  0  1   0 ] [vx]  k
%   [vy]       [0  0  0   1 ] [vy]

CV_A = [1  0  dt  0;
        0  1   0  dt;
        0  0   1   0;
        0  0   0   1];

fprintf('CV State Transition Matrix A (4x4):\n');
disp(CV_A);

% Initial state
CV_X0 = [50; 30; 2; 1];   % [x0, y0, vx0, vy0]
fprintf('CV Initial State [x, y, vx, vy]:\n');
fprintf('  x0=%.0f m,  y0=%.0f m,  vx0=%.1f m/s,  vy0=%.1f m/s\n\n', ...
        CV_X0(1), CV_X0(2), CV_X0(3), CV_X0(4));

% ---- Propagate: X(k+1) = A * X(k) ----
CV_X = zeros(4, N_steps);
CV_X(:,1) = CV_X0;
for k = 1:N_steps-1
    CV_X(:,k+1) = CV_A * CV_X(:,k);
end

% Print first few steps so you can see what's happening
fprintf('CV State History (first 6 steps):\n');
fprintf('  Step   t[s]    x[m]    y[m]   vx[m/s]  vy[m/s]\n');
for k = 1:6
    fprintf('   %3d   %4.1f  %7.2f %7.2f  %6.2f   %6.2f\n', ...
        k, t(k), CV_X(1,k), CV_X(2,k), CV_X(3,k), CV_X(4,k));
end
fprintf('\n');

%% ====================================================
%%  PART 2: CONSTANT ACCELERATION (CA) MODEL
%% ====================================================
fprintf('--- PART 2: Constant Acceleration (CA) Model ---\n\n');

% State vector: X = [x, y, vx, vy, ax, ay]
% ------------------------------------------
% x(k+1)  = x  + vx*dt + 0.5*ax*dt^2   <- position (kinematics)
% y(k+1)  = y  + vy*dt + 0.5*ay*dt^2
% vx(k+1) = vx + ax*dt                  <- velocity changes with acceleration
% vy(k+1) = vy + ay*dt
% ax(k+1) = ax                           <- acceleration stays constant
% ay(k+1) = ay

CA_A = [1  0  dt   0   0.5*dt^2    0      ;
        0  1   0  dt       0    0.5*dt^2  ;
        0  0   1   0      dt       0      ;
        0  0   0   1       0      dt      ;
        0  0   0   0       1       0      ;
        0  0   0   0       0       1      ];

fprintf('CA State Transition Matrix A (6x6):\n');
disp(CA_A);

% Initial state
CA_X0 = [50; 30; 2; 1; 0.05; -0.03];
fprintf('CA Initial State [x, y, vx, vy, ax, ay]:\n');
fprintf('  x0=%.0f m,  y0=%.0f m,  vx0=%.1f m/s,  vy0=%.1f m/s\n', ...
        CA_X0(1), CA_X0(2), CA_X0(3), CA_X0(4));
fprintf('  ax0=%.3f m/s^2,  ay0=%.3f m/s^2\n\n', CA_X0(5), CA_X0(6));

% ---- Propagate ----
CA_X = zeros(6, N_steps);
CA_X(:,1) = CA_X0;
for k = 1:N_steps-1
    CA_X(:,k+1) = CA_A * CA_X(:,k);
end

% Print first few steps
fprintf('CA State History (first 6 steps):\n');
fprintf('  Step   t[s]    x[m]    y[m]   vx[m/s]  vy[m/s]  ax    ay\n');
for k = 1:6
    fprintf('   %3d   %4.1f  %7.2f %7.2f  %6.3f   %6.3f  %.3f  %.3f\n', ...
        k, t(k), CA_X(1,k), CA_X(2,k), CA_X(3,k), CA_X(4,k), ...
        CA_X(5,k), CA_X(6,k));
end
fprintf('\n');

%% ====================================================
%%  PLOTS
%% ====================================================

% ---- Plot 1: Ship trajectory (X vs Y) ----
figure(1);
plot(CV_X(1,:), CV_X(2,:), 'b-',  'LineWidth', 2); hold on;
plot(CA_X(1,:), CA_X(2,:), 'r--', 'LineWidth', 2);
plot(CV_X(1,1), CV_X(2,1), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
xlabel('X Position [m]'); ylabel('Y Position [m]');
title('Ship Trajectory — World Frame');
legend('CV Model (straight line)', 'CA Model (curves due to acceleration)', ...
       'Start point', 'Location', 'best');
grid on;
annotation('textbox',[0.15 0.75 0.3 0.1], ...
    'String','CV: straight line (no acceleration)', ...
    'BackgroundColor','w','FontSize',8);

% ---- Plot 2: Position vs Time ----
figure(2);
subplot(2,1,1);
plot(t, CV_X(1,:), 'b-',  'LineWidth', 2); hold on;
plot(t, CA_X(1,:), 'r--', 'LineWidth', 2);
ylabel('X [m]'); title('Position vs Time');
legend('CV','CA'); grid on;

subplot(2,1,2);
plot(t, CV_X(2,:), 'b-',  'LineWidth', 2); hold on;
plot(t, CA_X(2,:), 'r--', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Y [m]');
legend('CV','CA'); grid on;

% ---- Plot 3: Velocity vs Time ----
figure(3);
subplot(2,1,1);
plot(t, CV_X(3,:), 'b-',  'LineWidth', 2); hold on;
plot(t, CA_X(3,:), 'r--', 'LineWidth', 2);
ylabel('Vx [m/s]'); title('Velocity vs Time');
legend('CV (constant)', 'CA (increasing)'); grid on;

subplot(2,1,2);
plot(t, CV_X(4,:), 'b-',  'LineWidth', 2); hold on;
plot(t, CA_X(4,:), 'r--', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Vy [m/s]');
legend('CV (constant)', 'CA (decreasing)'); grid on;

fprintf('ship_motion.m complete. 3 figures generated.\n');