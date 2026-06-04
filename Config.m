% =========================================================
% config.m — Simulation Parameters
%
% PURPOSE:
%   Define every constant used in the simulation.
%   Run this file first before anything else.
%
% HOW TO RUN:
%   Press F5 or type: config
% =========================================================

clear; clc;

fprintf('=========================================\n');
fprintf('  CONFIG — Simulation Parameters\n');
fprintf('=========================================\n\n');

%% ---- TIME SETTINGS ----
dt      = 0.5;          % Timestep [seconds]
T_total = 60;           % Total simulation time [seconds]
t       = 0:dt:T_total; % Time vector
N_steps = length(t);    % Total number of timesteps

fprintf('TIME SETTINGS:\n');
fprintf('  dt      = %.2f s  (how often we sample)\n', dt);
fprintf('  T_total = %.0f s  (simulation duration)\n', T_total);
fprintf('  N_steps = %d      (total timesteps)\n\n', N_steps);

%% ---- UAV / CAMERA SETTINGS ----
lambda  = 800;   % Focal length [pixels]
UAV_alt = 150;   % UAV altitude [metres]
UAV_x0  = 0;     % UAV start X [metres]
UAV_y0  = 0;     % UAV start Y [metres]
UAV_vx  = 0.5;   % UAV velocity X [m/s]
UAV_vy  = 0.2;   % UAV velocity Y [m/s]

fprintf('UAV / CAMERA SETTINGS:\n');
fprintf('  Focal length  lambda = %d px\n', lambda);
fprintf('  UAV altitude         = %d m\n', UAV_alt);
fprintf('  UAV start position   = (%.1f, %.1f) m\n', UAV_x0, UAV_y0);
fprintf('  UAV velocity         = (%.1f, %.1f) m/s\n\n', UAV_vx, UAV_vy);

%% ---- SHIP INITIAL STATE: CONSTANT VELOCITY MODEL ----
% State vector = [x, y, vx, vy]
CV_X0 = [50;    % x position [m]
          30;   % y position [m]
           2;   % velocity in x [m/s]
           1];  % velocity in y [m/s]

fprintf('SHIP INITIAL STATE — CV Model [x, y, vx, vy]:\n');
fprintf('  x0  = %.1f m\n',   CV_X0(1));
fprintf('  y0  = %.1f m\n',   CV_X0(2));
fprintf('  vx0 = %.1f m/s\n', CV_X0(3));
fprintf('  vy0 = %.1f m/s\n\n', CV_X0(4));

%% ---- SHIP INITIAL STATE: CONSTANT ACCELERATION MODEL ----
% State vector = [x, y, vx, vy, ax, ay]
CA_X0 = [50;     % x position [m]
          30;    % y position [m]
           2;    % velocity in x [m/s]
           1;    % velocity in y [m/s]
           0.05; % acceleration in x [m/s^2]
          -0.03];% acceleration in y [m/s^2]

fprintf('SHIP INITIAL STATE — CA Model [x, y, vx, vy, ax, ay]:\n');
fprintf('  x0  = %.1f m\n',    CA_X0(1));
fprintf('  y0  = %.1f m\n',    CA_X0(2));
fprintf('  vx0 = %.1f m/s\n',  CA_X0(3));
fprintf('  vy0 = %.1f m/s\n',  CA_X0(4));
fprintf('  ax0 = %.3f m/s^2\n',CA_X0(5));
fprintf('  ay0 = %.3f m/s^2\n\n',CA_X0(6));

%% ---- SHIP FEATURE POINTS ----
% 5 points on the ship deck, in body frame (relative to ship centre)
% Each column = one point = [X; Y; Z] offset in metres
ship_features = [-15  15  15  -15   0;   % X offsets
                 - 5  -5   5    5   0;   % Y offsets
                   0   0   0    0   0];  % Z offsets (flat deck)

N_features = size(ship_features, 2);

fprintf('SHIP FEATURE POINTS (%d points):\n', N_features);
fprintf('  Point   X[m]   Y[m]   Z[m]\n');
for f = 1:N_features
    fprintf('    %d     %4.0f   %4.0f   %4.0f\n', ...
        f, ship_features(1,f), ship_features(2,f), ship_features(3,f));
end

fprintf('\nconfig.m complete. All variables loaded into workspace.\n');