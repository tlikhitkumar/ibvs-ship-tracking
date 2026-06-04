% =========================================================
% camera_projection.m — Image Measurement Generator
%
% PURPOSE:
%   Show how 3D ship feature points get projected onto the
%   2D camera image plane.
%
%   Two steps happen here:
%
%   STEP 1 — Rigid Body Transform:
%       Pc = R * (Pw - d)
%       Convert world coordinates → camera frame coordinates
%       Pw = point in world,  d = UAV position,  R = rotation
%
%   STEP 2 — Perspective Projection:
%       u = lambda * (Xc / Zc)
%       v = lambda * (Yc / Zc)
%       Convert 3D camera coords → 2D image pixel coords
%
% HOW TO RUN:
%   Press F5 or type: camera_projection
%   (No inputs needed — everything is defined inside)
% =========================================================

clear; clc; close all;

fprintf('=========================================\n');
fprintf('  CAMERA PROJECTION — 3D to 2D\n');
fprintf('=========================================\n\n');

%% ---- Parameters (self-contained) ----
dt      = 0.5;
T_total = 60;
t       = 0:dt:T_total;
N_steps = length(t);

lambda  = 800;    % Focal length [pixels]
UAV_alt = 150;    % UAV altitude [metres]
UAV_x0  = 0;
UAV_y0  = 0;
UAV_vx  = 0.5;
UAV_vy  = 0.2;

% Ship initial state (CV model used here for simplicity)
X0 = [50; 30; 2; 1];

% State transition matrix (CV)
A = [1  0  dt  0;
     0  1   0  dt;
     0  0   1   0;
     0  0   0   1];

% Ship feature points (5 points, body frame offsets in metres)
ship_features = [-15  15  15  -15   0;
                 - 5  -5   5    5   0;
                   0   0   0    0   0];
N_features = size(ship_features, 2);

%% ---- Generate ship trajectory ----
X_hist = zeros(4, N_steps);
X_hist(:,1) = X0;
for k = 1:N_steps-1
    X_hist(:,k+1) = A * X_hist(:,k);
end

%% ====================================================
%%  STEP 1: ROTATION MATRIX R
%% ====================================================
fprintf('--- STEP 1: Rotation Matrix ---\n\n');

% UAV camera points straight down (nadir view).
% The camera Z-axis points downward (into the scene).
% So we need to flip World-Y and World-Z:
%
%   Camera X =  World X   (same)
%   Camera Y = -World Y   (flipped: image rows go downward)
%   Camera Z = -World Z   (flipped: camera looks down = -Z_world)

R = [1  0  0;
     0 -1  0;
     0  0 -1];

fprintf('Rotation Matrix R (nadir camera, pointing straight down):\n');
disp(R);
fprintf('  Row 1: Camera_X =  World_X\n');
fprintf('  Row 2: Camera_Y = -World_Y  (image rows increase downward)\n');
fprintf('  Row 3: Camera_Z = -World_Z  (camera looks toward ground)\n\n');

%% ====================================================
%%  STEP 2: PROJECTION — Single point example
%% ====================================================
fprintf('--- STEP 2: Single Point Projection Example (t=0) ---\n\n');

% UAV position at t=0
uav_pos = [UAV_x0; UAV_y0; UAV_alt];

% Take feature point 1 at t=0
ship_pos = [X_hist(1,1); X_hist(2,1); 0];   % ship at sea level
Pw = ship_pos + [ship_features(1,1); ship_features(2,1); ship_features(3,1)];

fprintf('  World point Pw            = [%.1f, %.1f, %.1f] m\n', Pw(1),Pw(2),Pw(3));
fprintf('  UAV position d            = [%.1f, %.1f, %.1f] m\n', uav_pos(1),uav_pos(2),uav_pos(3));

% Rigid body transform: Pc = R*(Pw - d)
Pc = R * (Pw - uav_pos);
fprintf('  Camera frame Pc = R*(Pw-d) = [%.2f, %.2f, %.2f]\n', Pc(1),Pc(2),Pc(3));

% Perspective projection
u = lambda * (Pc(1) / Pc(3));
v = lambda * (Pc(2) / Pc(3));
fprintf('  Image coords: u = lambda*(Xc/Zc) = %.1f px\n', u);
fprintf('                v = lambda*(Yc/Zc) = %.1f px\n\n', v);

%% ====================================================
%%  STEP 3: FULL SIMULATION — all features, all timesteps
%% ====================================================
fprintf('--- STEP 3: Full Projection (all features, all timesteps) ---\n\n');

u_hist = zeros(N_features, N_steps);
v_hist = zeros(N_features, N_steps);

for k = 1:N_steps

    % UAV position at step k
    uav_x = UAV_x0 + UAV_vx * (k-1) * dt;
    uav_y = UAV_y0 + UAV_vy * (k-1) * dt;
    d = [uav_x; uav_y; UAV_alt];

    % Ship position at step k
    ship_x = X_hist(1,k);
    ship_y = X_hist(2,k);

    for f = 1:N_features
        % World coordinates of this feature point
        Pw = [ship_x + ship_features(1,f);
              ship_y + ship_features(2,f);
              0      + ship_features(3,f)];

        % Transform to camera frame
        Pc = R * (Pw - d);

        Xc = Pc(1);
        Yc = Pc(2);
        Zc = Pc(3);

        if Zc <= 0
            u_hist(f,k) = NaN;
            v_hist(f,k) = NaN;
        else
            u_hist(f,k) = lambda * (Xc / Zc);
            v_hist(f,k) = lambda * (Yc / Zc);
        end
    end
end

% Print image coordinates at a few timesteps for feature 1
fprintf('  Image coords of Feature 1 at selected timesteps:\n');
fprintf('  Step   t[s]    u[px]    v[px]\n');
steps_to_show = [1, 10, 30, 61, 121];
for i = 1:length(steps_to_show)
    k = steps_to_show(i);
    if k <= N_steps
        fprintf('   %3d   %4.1f   %7.2f  %7.2f\n', ...
            k, t(k), u_hist(1,k), v_hist(1,k));
    end
end

%% ====================================================
%%  PLOTS
%% ====================================================

% ---- Plot 1: Image feature trajectories (u-v plane) ----
figure(1);
colors = lines(N_features);
hold on;
for f = 1:N_features
    plot(u_hist(f,:), v_hist(f,:), '-', 'Color', colors(f,:), 'LineWidth', 1.8);
    plot(u_hist(f,1), v_hist(f,1), 'o', 'Color', colors(f,:), ...
         'MarkerSize', 8, 'MarkerFaceColor', colors(f,:));
    plot(u_hist(f,end), v_hist(f,end), 's', 'Color', colors(f,:), ...
         'MarkerSize', 8, 'MarkerFaceColor', colors(f,:));
end
xlabel('u [pixels]'); ylabel('v [pixels]');
title('Image Feature Trajectories (u-v Plane)');
legend(arrayfun(@(x) sprintf('Feature %d', x), 1:N_features, ...
       'UniformOutput', false), 'Location', 'best');
grid on;
annotation('textbox',[0.15 0.8 0.35 0.08], ...
    'String',{'Circle = start,  Square = end'}, ...
    'BackgroundColor','w','FontSize',8);

% ---- Plot 2: u and v coordinates vs time for all features ----
figure(2);
subplot(2,1,1);
hold on;
for f = 1:N_features
    plot(t, u_hist(f,:), 'LineWidth', 1.5);
end
ylabel('u [pixels]');
title('Image Coordinates vs Time');
legend(arrayfun(@(x) sprintf('Feature %d',x), 1:N_features, ...
       'UniformOutput',false), 'Location','best');
grid on;

subplot(2,1,2);
hold on;
for f = 1:N_features
    plot(t, v_hist(f,:), 'LineWidth', 1.5);
end
xlabel('Time [s]'); ylabel('v [pixels]');
grid on;

% ---- Plot 3: Depth Z (distance from camera to feature) ----
figure(3);
hold on;
for k = 1:N_steps
    uav_x = UAV_x0 + UAV_vx * (k-1) * dt;
    uav_y = UAV_y0 + UAV_vy * (k-1) * dt;
    d = [uav_x; uav_y; UAV_alt];
    Pw_centre = [X_hist(1,k); X_hist(2,k); 0];
    Pc_centre = R * (Pw_centre - d);
    Z_centre(k) = Pc_centre(3);
end
plot(t, Z_centre, 'k-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Depth Z [m]');
title('Camera Depth to Ship Centre vs Time');
grid on;
annotation('textbox',[0.15 0.75 0.55 0.1], ...
    'String',{'Larger Z = ship appears smaller in image', ...
              'Smaller Z = ship appears larger in image'}, ...
    'BackgroundColor','w','FontSize',8);

fprintf('\ncamera_projection.m complete. 3 figures generated.\n');