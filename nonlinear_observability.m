% =========================================================
% nonlinear_observability.m — LTI (constant C) vs
%                              LTV (Jacobian H) Observability
%
% PURPOSE:
%   Compare the old approach (constant C, flat rank forever)
%   against the new approach (Jacobian H, rebuilt each step).
%
% HOW TO RUN:
%   Press F5 or type: nonlinear_observability
%   (Self-contained — no other files required)
% =========================================================

clear; clc; close all;

fprintf('=========================================\n');
fprintf('  NONLINEAR OBSERVABILITY (LTI vs LTV)\n');
fprintf('=========================================\n\n');

%% ---- Parameters ----
dt = 0.5;  T_total = 60;  t = 0:dt:T_total;  N_steps = length(t);
fx = 800;  fy = 800;  z = 150;

A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
n = 4;

h      = @(X) [fx*(X(1)/z); fy*(X(2)/z)];
H_func = @(X) [fx/z 0 0 0; 0 fy/z 0 0];
C_old  = [1 0 0 0; 0 1 0 0];

%% ---- True trajectory ----
X_hist = zeros(n, N_steps);
X_hist(:,1) = [50; 30; 2; 1];
for k = 1:N_steps-1
    X_hist(:,k+1) = A * X_hist(:,k);
end

%% ====================================================
%%  LTI: constant C (old approach)
%% ====================================================
fprintf('--- LTI: constant C ---\n\n');

O_LTI = [C_old; C_old*A; C_old*A^2; C_old*A^3];
r_LTI = rank(O_LTI);
c_LTI = cond(O_LTI);
rank_LTI_t = r_LTI * ones(1,N_steps);
cond_LTI_t = c_LTI * ones(1,N_steps);

Wo_LTI = zeros(n,n);
det_LTI = zeros(1,N_steps);
eig_LTI = zeros(1,N_steps);
for k = 0:N_steps-1
    Ak = A^k;
    Wo_LTI = Wo_LTI + Ak'*C_old'*C_old*Ak;
    det_LTI(k+1) = det(Wo_LTI);
    eig_LTI(k+1) = min(eig(Wo_LTI));
end

fprintf('rank(O)=%d/4   cond(O)=%.3e\n', r_LTI, c_LTI);
fprintf('Final det(Wo)=%.3e   min_eig=%.3e\n\n', det_LTI(end), eig_LTI(end));

%% ====================================================
%%  LTV: Jacobian H (new approach)
%% ====================================================
fprintf('--- LTV: Jacobian H(k) ---\n\n');

rank_LTV_t = zeros(1,N_steps);
cond_LTV_t = zeros(1,N_steps);

for k = 1:N_steps
    H_k = H_func(X_hist(:,k));
    O_k = [H_k; H_k*A; H_k*A^2; H_k*A^3];
    rank_LTV_t(k) = rank(O_k);
    cond_LTV_t(k) = cond(O_k);
end

Wo_LTV = zeros(n,n);
det_LTV = zeros(1,N_steps);
eig_LTV = zeros(1,N_steps);
for k = 0:N_steps-1
    Ak  = A^k;
    H_k = H_func(X_hist(:,k+1));
    Wo_LTV = Wo_LTV + Ak'*H_k'*H_k*Ak;
    det_LTV(k+1) = det(Wo_LTV);
    eig_LTV(k+1) = min(eig(Wo_LTV));
end

fprintf('rank(O) range: %d to %d\n', min(rank_LTV_t), max(rank_LTV_t));
fprintf('Final det(Wo)=%.3e   min_eig=%.3e\n\n', det_LTV(end), eig_LTV(end));

fprintf('Note: with z constant, H(k) does not actually change over\n');
fprintf('time, so LTV results equal LTI here. This becomes genuinely\n');
fprintf('time-varying once z changes OR once UAV path changes the\n');
fprintf('relative geometry (next phase of the project).\n\n');

%% ---- Plots ----
c_LTI = [0.15 0.45 0.80];
c_LTV = [0.55 0.18 0.55];

figure(1); set(gcf,'Name','NLObs Fig 1 - rank(O)');
plot(t,rank_LTI_t,'-','Color',c_LTI,'LineWidth',2.5); hold on;
plot(t,rank_LTV_t,'--','Color',c_LTV,'LineWidth',2.5);
yline(4,':','Full obs (n=4)','LabelHorizontalAlignment','left');
xlabel('Time [s]'); ylabel('rank(O(k))');
title('NLObs Fig 1 - LTI vs LTV Rank');
legend('LTI: constant C','LTV: Jacobian H(k)','Location','best');
ylim([0 6]); yticks(0:1:6); grid on;

figure(2); set(gcf,'Name','NLObs Fig 2 - cond(O)');
semilogy(t,cond_LTI_t,'-','Color',c_LTI,'LineWidth',2); hold on;
semilogy(t,cond_LTV_t,'--','Color',c_LTV,'LineWidth',2);
xlabel('Time [s]'); ylabel('cond(O) [log]');
title('NLObs Fig 2 - LTI vs LTV Condition Number');
legend('LTI','LTV','Location','best'); grid on;

figure(3); set(gcf,'Name','NLObs Fig 3 - Gramian comparison');
subplot(2,1,1);
plot(t,det_LTI,'-','Color',c_LTI,'LineWidth',2); hold on;
plot(t,det_LTV,'--','Color',c_LTV,'LineWidth',2);
ylabel('det(Wo)'); title('NLObs Fig 3 - Gramian Determinant');
legend('LTI','LTV','Location','best'); grid on;
subplot(2,1,2);
semilogy(t,eig_LTI,'-','Color',c_LTI,'LineWidth',2); hold on;
semilogy(t,eig_LTV,'--','Color',c_LTV,'LineWidth',2);
xlabel('Time [s]'); ylabel('min eig(Wo) [log]');
legend('LTI','LTV','Location','best'); grid on;

fprintf('nonlinear_observability.m complete. 3 figures generated.\n');