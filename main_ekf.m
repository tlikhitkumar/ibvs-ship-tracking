% =========================================================
% main_ekf.m — Phase 1 Master Script
%              EKF + Nonlinear Observability (LTI vs LTV)
%
% PURPOSE:
%   Run the full Phase 1 pipeline in one go.
%   UAV still flies its simple straight patrol here —
%   trajectory comparison (straight vs zig-zag) is Phase 2,
%   a separate set of files, not mixed in here.
%
% HOW TO RUN:
%   Press F5 or type: main_ekf
% =========================================================

clear; clc; close all;

fprintf('============================================\n');
fprintf('  PHASE 1: EKF + Nonlinear Observability\n');
fprintf('============================================\n\n');

%% ---- Parameters ----
dt = 0.5;  T_total = 60;  t = 0:dt:T_total;  N_steps = length(t);
fx = 800;  fy = 800;  z = 150;

A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
n = 4;

h      = @(X) [fx*(X(1)/z); fy*(X(2)/z)];
H_func = @(X) [fx/z 0 0 0; 0 fy/z 0 0];
C_old  = [1 0 0 0; 0 1 0 0];

Q = diag([0.1, 0.1, 0.05, 0.05]);
R = diag([4, 4]);

rng(42);

%% ---- True trajectory ----
fprintf('[1/5] True ship trajectory...\n');
X_true = zeros(n, N_steps);
X_true(:,1) = [50; 30; 2; 1];
for k = 1:N_steps-1
    X_true(:,k+1) = A * X_true(:,k);
end

%% ---- Noisy measurements ----
fprintf('[2/5] Noisy camera measurements...\n');
z_meas = zeros(2, N_steps);
for k = 1:N_steps
    z_meas(:,k) = h(X_true(:,k)) + sqrt(R)*randn(2,1);
end

%% ---- EKF ----
fprintf('[3/5] Running EKF...\n');
X_est  = zeros(n, N_steps);
P_arr  = zeros(n, n, N_steps);
innov  = zeros(2, N_steps);
X_est(:,1)   = [48; 28; 1.8; 0.8];
P_arr(:,:,1) = 10*eye(n);

for k = 1:N_steps-1
    X_pred = A * X_est(:,k);
    P_pred = A * P_arr(:,:,k) * A' + Q;
    H      = H_func(X_pred);
    z_pred = h(X_pred);
    r      = z_meas(:,k+1) - z_pred;
    S      = H * P_pred * H' + R;
    K      = P_pred * H' * (S \ eye(2));
    X_est(:,k+1)   = X_pred + K * r;
    P_arr(:,:,k+1) = (eye(n) - K*H) * P_pred;
    innov(:,k+1)   = r;
end
pos_err = sqrt((X_true(1,:)-X_est(1,:)).^2 + (X_true(2,:)-X_est(2,:)).^2);
fprintf('    Final position error: %.4f m\n', pos_err(end));

%% ---- LTI observability ----
fprintf('[4/5] LTI observability (constant C)...\n');
O_LTI = [C_old; C_old*A; C_old*A^2; C_old*A^3];
r_LTI = rank(O_LTI);  c_LTI = cond(O_LTI);
rank_LTI_t = r_LTI*ones(1,N_steps);
cond_LTI_t = c_LTI*ones(1,N_steps);

Wo_LTI = zeros(n,n); det_LTI = zeros(1,N_steps); eig_LTI = zeros(1,N_steps);
for k = 0:N_steps-1
    Ak = A^k;
    Wo_LTI = Wo_LTI + Ak'*C_old'*C_old*Ak;
    det_LTI(k+1) = det(Wo_LTI);
    eig_LTI(k+1) = min(eig(Wo_LTI));
end
fprintf('    rank=%d/4  cond(O)=%.3e\n', r_LTI, c_LTI);

%% ---- LTV observability ----
fprintf('[5/5] LTV observability (Jacobian H)...\n');
rank_LTV_t = zeros(1,N_steps);
cond_LTV_t = zeros(1,N_steps);
for k = 1:N_steps
    H_k = H_func(X_true(:,k));
    O_k = [H_k; H_k*A; H_k*A^2; H_k*A^3];
    rank_LTV_t(k) = rank(O_k);
    cond_LTV_t(k) = cond(O_k);
end

Wo_LTV = zeros(n,n); det_LTV = zeros(1,N_steps); eig_LTV = zeros(1,N_steps);
for k = 0:N_steps-1
    Ak  = A^k;
    H_k = H_func(X_true(:,k+1));
    Wo_LTV = Wo_LTV + Ak'*H_k'*H_k*Ak;
    det_LTV(k+1) = det(Wo_LTV);
    eig_LTV(k+1) = min(eig(Wo_LTV));
end
fprintf('    rank range: %d to %d\n\n', min(rank_LTV_t), max(rank_LTV_t));

%% ====================================================
%%  PLOTS  (12 figures total)
%% ====================================================
c_true=[0.15 0.45 0.80]; c_est=[0.85 0.33 0.10];
c_inn=[0.13 0.55 0.13];  c_LTI=[0.15 0.45 0.80]; c_LTV=[0.55 0.18 0.55];

% Fig 1: True trajectory
figure(1); set(gcf,'Name','Fig 1 - True Trajectory');
plot(X_true(1,:),X_true(2,:),'-','Color',c_true,'LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]'); title('Fig 1 - True Ship Trajectory'); grid on;

% Fig 2: EKF trajectory comparison
figure(2); set(gcf,'Name','Fig 2 - EKF Trajectory');
plot(X_true(1,:),X_true(2,:),'-','Color',c_true,'LineWidth',2); hold on;
plot(X_est(1,:),X_est(2,:),'--','Color',c_est,'LineWidth',2);
plot(X_true(1,1),X_true(2,1),'ko','MarkerSize',8,'MarkerFaceColor','k');
xlabel('X [m]'); ylabel('Y [m]');
title('Fig 2 - EKF: True vs Estimated Trajectory');
legend('True','EKF estimate','Start','Location','best'); grid on;

% Fig 3: EKF state estimates
figure(3); set(gcf,'Name','Fig 3 - EKF States');
labels = {'x [m]','y [m]','vx [m/s]','vy [m/s]'};
for i=1:4
    subplot(2,2,i);
    plot(t,X_true(i,:),'-','Color',c_true,'LineWidth',2); hold on;
    plot(t,X_est(i,:),'--','Color',c_est,'LineWidth',2);
    ylabel(labels{i}); legend('True','EKF'); grid on;
    if i>=3; xlabel('Time [s]'); end
end
sgtitle('Fig 3 - EKF State Estimates vs True States');

% Fig 4: Position error
figure(4); set(gcf,'Name','Fig 4 - EKF Error');
plot(t,pos_err,'-','Color',c_est,'LineWidth',2);
xlabel('Time [s]'); ylabel('Position error [m]');
title('Fig 4 - EKF Position Error (should converge)'); grid on;

% Fig 5: Innovation
figure(5); set(gcf,'Name','Fig 5 - Innovation');
subplot(2,1,1);
plot(t,innov(1,:),'-','Color',c_inn,'LineWidth',1.5); hold on; yline(0,'k--');
ylabel('r_u [px]'); title('Fig 5 - EKF Innovation'); grid on;
subplot(2,1,2);
plot(t,innov(2,:),'-','Color',c_inn,'LineWidth',1.5); hold on; yline(0,'k--');
xlabel('Time [s]'); ylabel('r_v [px]'); grid on;

% Fig 6: Covariance
figure(6); set(gcf,'Name','Fig 6 - Covariance');
for i=1:n; P_d(i,:)=squeeze(P_arr(i,i,:))'; end
semilogy(t,P_d(1,:),'-','LineWidth',2); hold on;
semilogy(t,P_d(2,:),'--','LineWidth',2);
semilogy(t,P_d(3,:),':','LineWidth',2);
semilogy(t,P_d(4,:),'-.','LineWidth',2);
xlabel('Time [s]'); ylabel('Variance [log]');
title('Fig 6 - EKF Covariance (should decrease)');
legend('P_x','P_y','P_{vx}','P_{vy}','Location','best'); grid on;

% Fig 7: LTI rank
figure(7); set(gcf,'Name','Fig 7 - LTI Rank');
plot(t,rank_LTI_t,'-','Color',c_LTI,'LineWidth',2.5);
yline(4,':','Full obs');
xlabel('Time [s]'); ylabel('rank(O)');
title('Fig 7 - LTI Rank (constant C - flat)');
ylim([0 6]); yticks(0:1:6); grid on;

% Fig 8: LTV rank
figure(8); set(gcf,'Name','Fig 8 - LTV Rank');
plot(t,rank_LTV_t,'--','Color',c_LTV,'LineWidth',2.5);
yline(4,':','Full obs');
xlabel('Time [s]'); ylabel('rank(O(k))');
title('Fig 8 - LTV Rank (Jacobian H)');
ylim([0 6]); yticks(0:1:6); grid on;

% Fig 9: LTI vs LTV rank overlay
figure(9); set(gcf,'Name','Fig 9 - LTI vs LTV Rank');
plot(t,rank_LTI_t,'-','Color',c_LTI,'LineWidth',2.5); hold on;
plot(t,rank_LTV_t,'--','Color',c_LTV,'LineWidth',2.5);
yline(4,':','Color',[0.4 0.4 0.4]);
xlabel('Time [s]'); ylabel('rank(O(k))');
title('Fig 9 - LTI vs LTV Rank Comparison');
legend('LTI: constant C','LTV: Jacobian H','Location','best');
ylim([0 6]); yticks(0:1:6); grid on;

% Fig 10: cond(O) comparison
figure(10); set(gcf,'Name','Fig 10 - cond(O) Comparison');
semilogy(t,cond_LTI_t,'-','Color',c_LTI,'LineWidth',2); hold on;
semilogy(t,cond_LTV_t,'--','Color',c_LTV,'LineWidth',2);
xlabel('Time [s]'); ylabel('cond(O) [log]');
title('Fig 10 - LTI vs LTV Condition Number');
legend('LTI','LTV','Location','best'); grid on;

% Fig 11: Gramian det
figure(11); set(gcf,'Name','Fig 11 - Gramian det(Wo)');
plot(t,det_LTI,'-','Color',c_LTI,'LineWidth',2); hold on;
plot(t,det_LTV,'--','Color',c_LTV,'LineWidth',2);
xlabel('Time [s]'); ylabel('det(Wo)');
title('Fig 11 - Gramian Determinant: LTI vs LTV');
legend('LTI','LTV','Location','best'); grid on;

% Fig 12: KEY comparison grid
figure(12); set(gcf,'Name','Fig 12 - KEY Full Comparison');
set(gcf,'Position',[50 50 1000 600]);
subplot(2,2,1);
plot(t,rank_LTI_t,'-','Color',c_LTI,'LineWidth',2); hold on;
plot(t,rank_LTV_t,'--','Color',c_LTV,'LineWidth',2);
ylabel('rank(O)'); title('Rank: LTI vs LTV');
legend('LTI','LTV','Location','best'); ylim([0 6]); grid on;
subplot(2,2,2);
semilogy(t,cond_LTI_t,'-','Color',c_LTI,'LineWidth',2); hold on;
semilogy(t,cond_LTV_t,'--','Color',c_LTV,'LineWidth',2);
ylabel('cond(O) [log]'); title('Condition Number');
legend('LTI','LTV','Location','best'); grid on;
subplot(2,2,3);
plot(t,pos_err,'-','Color',c_est,'LineWidth',2);
xlabel('Time [s]'); ylabel('Error [m]');
title('EKF Position Error'); grid on;
subplot(2,2,4);
semilogy(t,eig_LTI,'-','Color',c_LTI,'LineWidth',2); hold on;
semilogy(t,eig_LTV,'--','Color',c_LTV,'LineWidth',2);
xlabel('Time [s]'); ylabel('min eig(Wo) [log]');
title('Gramian Min Eigenvalue');
legend('LTI','LTV','Location','best'); grid on;
sgtitle('Fig 12 - Phase 1 Summary: EKF + LTI vs LTV Observability');

%% ---- Final summary ----
fprintf('============================================\n');
fprintf('  PHASE 1 RESULTS\n');
fprintf('============================================\n');
fprintf('  LTI (constant C):  rank=%d/4  cond(O)=%.3e\n', r_LTI, c_LTI);
fprintf('  LTV (Jacobian H):  rank range %d to %d\n', min(rank_LTV_t), max(rank_LTV_t));
fprintf('  EKF final position error: %.4f m\n', pos_err(end));
fprintf('  12 figures generated.\n');
fprintf('============================================\n');