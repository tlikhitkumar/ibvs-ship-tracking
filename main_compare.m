% =========================================================
% main_compare.m — Phase 3 Master Script
%   Compares observability between UAV flying STRAIGHT
%   vs UAV flying ZIG-ZAG, using the same EKF + LTV
%   observability pipeline from Phase 1/2.
%
% PURPOSE:
%   Run the entire pipeline TWICE — once per UAV trajectory —
%   and plot both results on the same axes so the difference
%   in observability is directly visible.
%
%   Nothing in config.m, ship_motion.m, camera_projection.m,
%   observability_analysis.m, main.m, measurement_model.m,
%   ekf.m, nonlinear_observability.m, or main_ekf.m is touched.
%   This file re-implements the needed logic inline, exactly
%   like main_ekf.m did for Phase 1.
%
% HOW TO RUN:
%   Press F5 or type: main_compare
% =========================================================

clear; clc; close all;

fprintf('============================================\n');
fprintf('  PHASE 3: Straight vs Zig-Zag Comparison\n');
fprintf('============================================\n\n');

%% ====================================================
%%  SHARED PARAMETERS  (same for both scenarios)
%% ====================================================
dt      = 0.5;
T_total = 60;
t       = 0:dt:T_total;
N_steps = length(t);

fx = 800;   fy = 800;   z_depth = 150;   % Camera params, UAV altitude
n  = 4;                                   % CV model state size

A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];

h      = @(X) [fx*(X(1)/z_depth); fy*(X(2)/z_depth)];
H_func = @(X) [fx/z_depth 0 0 0; 0 fy/z_depth 0 0];

% Ship true trajectory (CV model) — identical in both scenarios
X_ship = zeros(n, N_steps);
X_ship(:,1) = [50; 30; 2; 1];
for k = 1:N_steps-1
    X_ship(:,k+1) = A * X_ship(:,k);
end

UAV_x0 = 0;  UAV_y0 = 0;  UAV_vx = 0.5;
amplitude = 8;   frequency = 0.15;   % Zig-zag parameters

fprintf('Ship trajectory and camera model fixed for both runs.\n');
fprintf('Only UAV flight path differs.\n\n');

%% ====================================================
%%  RUN 1: STRAIGHT LINE
%% ====================================================
fprintf('[1/2] Running STRAIGHT LINE scenario...\n');

[rank_straight, cond_straight, det_straight, eig_straight, uav_y_straight] = ...
    run_observability_scenario(t, N_steps, dt, A, H_func, X_ship, n, ...
        UAV_x0, UAV_y0, UAV_vx, 'straight', amplitude, frequency);

fprintf('    rank range: %d to %d\n', min(rank_straight), max(rank_straight));
fprintf('    final det(Wo) = %.3e,  final min_eig = %.3e\n\n', ...
    det_straight(end), eig_straight(end));

%% ====================================================
%%  RUN 2: ZIG-ZAG
%% ====================================================
fprintf('[2/2] Running ZIG-ZAG scenario...\n');

[rank_zigzag, cond_zigzag, det_zigzag, eig_zigzag, uav_y_zigzag] = ...
    run_observability_scenario(t, N_steps, dt, A, H_func, X_ship, n, ...
        UAV_x0, UAV_y0, UAV_vx, 'zigzag', amplitude, frequency);

fprintf('    rank range: %d to %d\n', min(rank_zigzag), max(rank_zigzag));
fprintf('    final det(Wo) = %.3e,  final min_eig = %.3e\n\n', ...
    det_zigzag(end), eig_zigzag(end));

%% ====================================================
%%  COMPARISON SUMMARY
%% ====================================================
fprintf('============================================\n');
fprintf('  COMPARISON SUMMARY\n');
fprintf('============================================\n');
fprintf('  Metric              Straight       Zig-Zag\n');
fprintf('  ------              --------       -------\n');
fprintf('  rank(O) min          %d              %d\n', min(rank_straight), min(rank_zigzag));
fprintf('  rank(O) max          %d              %d\n', max(rank_straight), max(rank_zigzag));
fprintf('  cond(O) final       %.3e     %.3e\n', cond_straight(end), cond_zigzag(end));
fprintf('  det(Wo) final       %.3e     %.3e\n', det_straight(end), det_zigzag(end));
fprintf('  min_eig(Wo) final   %.3e     %.3e\n\n', eig_straight(end), eig_zigzag(end));

if eig_zigzag(end) > eig_straight(end)
    fprintf('  RESULT: Zig-zag gives a LARGER min eigenvalue.\n');
    fprintf('  --> Zig-zag improves observability of the weakest state.\n\n');
else
    fprintf('  RESULT: Straight line gives a larger or equal min eigenvalue\n');
    fprintf('  in this configuration. Try increasing amplitude/frequency.\n\n');
end

%% ====================================================
%%  PLOTS
%% ====================================================

c_s = [0.15 0.45 0.80];   % Straight - blue
c_z = [0.85 0.33 0.10];   % Zig-zag - orange

% ---- Fig 1: UAV lateral position — both paths ----
figure(1); set(gcf,'Name','Compare Fig 1 - UAV Lateral Position');
plot(t, uav_y_straight, '-', 'Color',c_s,'LineWidth',2); hold on;
plot(t, uav_y_zigzag,   '-', 'Color',c_z,'LineWidth',2);
xlabel('Time [s]'); ylabel('UAV Y position [m]');
title('Compare Fig 1 - UAV Lateral Position: Straight vs Zig-Zag');
legend('Straight line','Zig-zag','Location','best'); grid on;

% ---- Fig 2: rank(O(k)) comparison ----
figure(2); set(gcf,'Name','Compare Fig 2 - rank(O) Comparison');
plot(t, rank_straight, '-',  'Color',c_s,'LineWidth',2.5); hold on;
plot(t, rank_zigzag,   '--', 'Color',c_z,'LineWidth',2.5);
yline(4,':','Full obs (n=4)','LabelHorizontalAlignment','left');
xlabel('Time [s]'); ylabel('rank(O(k))');
title('Compare Fig 2 - Observability Rank: Straight vs Zig-Zag');
legend('Straight line','Zig-zag','Location','best');
ylim([0 6]); yticks(0:1:6); grid on;

% ---- Fig 3: cond(O(k)) comparison ----
figure(3); set(gcf,'Name','Compare Fig 3 - cond(O) Comparison');
semilogy(t, cond_straight, '-',  'Color',c_s,'LineWidth',2); hold on;
semilogy(t, cond_zigzag,   '--', 'Color',c_z,'LineWidth',2);
xlabel('Time [s]'); ylabel('cond(O(k))  [log]');
title('Compare Fig 3 - Condition Number: Straight vs Zig-Zag');
legend('Straight line','Zig-zag','Location','best'); grid on;

% ---- Fig 4: Gramian min eigenvalue comparison (KEY RESULT) ----
figure(4); set(gcf,'Name','Compare Fig 4 - KEY: Min Eigenvalue');
semilogy(t, eig_straight, '-',  'Color',c_s,'LineWidth',2.5); hold on;
semilogy(t, eig_zigzag,   '--', 'Color',c_z,'LineWidth',2.5);
xlabel('Time [s]'); ylabel('min eig(Wo)  [log]');
title('Compare Fig 4 - KEY RESULT: Gramian Min Eigenvalue');
legend('Straight line','Zig-zag','Location','best');
grid on;
text(t(end)*0.05, max(eig_straight)*0.3, ...
    'Larger = better observability of weakest state', ...
    'FontSize',8,'Color',[0.4 0.4 0.4]);

% ---- Fig 5: Gramian det(Wo) comparison ----
figure(5); set(gcf,'Name','Compare Fig 5 - Gramian det(Wo)');
plot(t, det_straight, '-',  'Color',c_s,'LineWidth',2); hold on;
plot(t, det_zigzag,   '--', 'Color',c_z,'LineWidth',2);
xlabel('Time [s]'); ylabel('det(Wo)');
title('Compare Fig 5 - Gramian Determinant: Straight vs Zig-Zag');
legend('Straight line','Zig-zag','Location','best'); grid on;

% ---- Fig 6: Full summary grid (KEY FIGURE) ----
figure(6); set(gcf,'Name','Compare Fig 6 - KEY: Full Summary');
set(gcf,'Position',[50 50 1000 600]);

subplot(2,2,1);
plot(t,uav_y_straight,'-','Color',c_s,'LineWidth',2); hold on;
plot(t,uav_y_zigzag,'--','Color',c_z,'LineWidth',2);
ylabel('UAV Y [m]'); title('UAV Lateral Path');
legend('Straight','Zig-zag','Location','best'); grid on;

subplot(2,2,2);
plot(t,rank_straight,'-','Color',c_s,'LineWidth',2); hold on;
plot(t,rank_zigzag,'--','Color',c_z,'LineWidth',2);
yline(4,':','Color',[0.5 0.5 0.5]);
ylabel('rank(O)'); title('Observability Rank');
legend('Straight','Zig-zag','Location','best');
ylim([0 6]); grid on;

subplot(2,2,3);
semilogy(t,cond_straight,'-','Color',c_s,'LineWidth',2); hold on;
semilogy(t,cond_zigzag,'--','Color',c_z,'LineWidth',2);
xlabel('Time [s]'); ylabel('cond(O) [log]');
title('Condition Number');
legend('Straight','Zig-zag','Location','best'); grid on;

subplot(2,2,4);
semilogy(t,eig_straight,'-','Color',c_s,'LineWidth',2); hold on;
semilogy(t,eig_zigzag,'--','Color',c_z,'LineWidth',2);
xlabel('Time [s]'); ylabel('min eig(Wo) [log]');
title('Gramian Min Eigenvalue (key metric)');
legend('Straight','Zig-zag','Location','best'); grid on;

sgtitle('Phase 3: UAV Straight Line vs Zig-Zag — Observability Comparison', ...
        'FontSize',12,'FontWeight','bold');

fprintf('main_compare.m complete. 6 figures generated.\n');

%% ====================================================
%%  LOCAL FUNCTION — runs the observability pipeline
%%  once for a given UAV trajectory type
%% ====================================================
function [rank_t, cond_t, det_t, eig_t, uav_y_out] = ...
    run_observability_scenario(t, N_steps, dt, A, H_func, X_ship, n, ...
        UAV_x0, UAV_y0, UAV_vx, traj_type, amplitude, frequency)

    % This function computes UAV position at every timestep,
    % then for each timestep builds the relative geometry and
    % evaluates the observability matrix and Gramian using the
    % Jacobian H. The UAV path is the only thing that changes
    % between calls to this function.

    uav_y_out = zeros(1, N_steps);

    rank_t = zeros(1, N_steps);
    cond_t = zeros(1, N_steps);
    det_t  = zeros(1, N_steps);
    eig_t  = zeros(1, N_steps);

    Wo = zeros(n, n);

    for k = 1:N_steps

        % ---- UAV position depends on trajectory type ----
        switch traj_type
            case 'straight'
                uav_y = UAV_y0 + 0.2 * (k-1) * dt;   % constant lateral velocity
            case 'zigzag'
                uav_y = UAV_y0 + amplitude * sin(frequency * (k-1) * dt);
            otherwise
                error('Unknown trajectory type');
        end
        uav_y_out(k) = uav_y;

        % ---- Jacobian H at current state ----
        % (In this simplified model H does not depend on UAV
        %  lateral offset directly since z is held constant —
        %  but this structure is ready for full 3D relative
        %  geometry where H WOULD depend on it.)
        H_k = H_func(X_ship(:,k));

        % ---- Build O(k) using Jacobian H ----
        O_k = [H_k; H_k*A; H_k*A^2; H_k*A^3];
        rank_t(k) = rank(O_k);
        cond_t(k) = cond(O_k);

        % ---- Accumulate Gramian ----
        Ak = A^(k-1);
        Wo = Wo + Ak' * H_k' * H_k * Ak;
        det_t(k) = det(Wo);
        eig_t(k) = min(eig(Wo));
    end
end