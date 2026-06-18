% =========================================================
% ekf.m — Extended Kalman Filter
%
% PURPOSE:
%   Track ship state [x,y,vx,vy] using noisy pixel measurements.
%   Two stages every step: PREDICT then UPDATE.
%
% HOW TO RUN:
%   Press F5 or type: ekf
%   (Self-contained — no other files required)
% =========================================================

clear; clc; close all;

fprintf('=========================================\n');
fprintf('  EXTENDED KALMAN FILTER (EKF)\n');
fprintf('=========================================\n\n');

%% ---- Parameters ----
dt = 0.5;  T_total = 60;  t = 0:dt:T_total;  N_steps = length(t);
fx = 800;  fy = 800;  z = 150;

A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
n = 4;

Q = diag([0.1, 0.1, 0.05, 0.05]);   % Process noise
R = diag([4, 4]);                    % Measurement noise (2px std)

h      = @(X) [fx*(X(1)/z); fy*(X(2)/z)];
H_func = @(X) [fx/z 0 0 0; 0 fy/z 0 0];

%% ---- True trajectory ----
X_true = zeros(n, N_steps);
X_true(:,1) = [50; 30; 2; 1];
for k = 1:N_steps-1
    X_true(:,k+1) = A * X_true(:,k);
end

%% ---- Noisy measurements ----
rng(42);
z_meas = zeros(2, N_steps);
for k = 1:N_steps
    z_meas(:,k) = h(X_true(:,k)) + sqrt(R)*randn(2,1);
end

%% ---- EKF loop ----
X_est  = zeros(n, N_steps);
P_arr  = zeros(n, n, N_steps);
innov  = zeros(2, N_steps);

X_est(:,1)   = [48; 28; 1.8; 0.8];   % Deliberately wrong start
P_arr(:,:,1) = 10*eye(n);

fprintf('Initial estimate: [%.1f, %.1f, %.1f, %.1f]\n', X_est(1,1),X_est(2,1),X_est(3,1),X_est(4,1));
fprintf('True initial:     [%.1f, %.1f, %.1f, %.1f]\n\n', X_true(1,1),X_true(2,1),X_true(3,1),X_true(4,1));

for k = 1:N_steps-1
    % PREDICT
    X_pred = A * X_est(:,k);
    P_pred = A * P_arr(:,:,k) * A' + Q;

    % UPDATE
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
fprintf('Final position error: %.4f m\n\n', pos_err(end));

%% ---- Plots ----
c_true = [0.15 0.45 0.80];
c_est  = [0.85 0.33 0.10];
c_inn  = [0.13 0.55 0.13];

figure(1); set(gcf,'Name','EKF Fig 1 - Trajectory');
plot(X_true(1,:),X_true(2,:),'-','Color',c_true,'LineWidth',2); hold on;
plot(X_est(1,:),X_est(2,:),'--','Color',c_est,'LineWidth',2);
plot(X_true(1,1),X_true(2,1),'ko','MarkerSize',8,'MarkerFaceColor','k');
xlabel('X [m]'); ylabel('Y [m]');
title('EKF Fig 1 - True vs Estimated Trajectory');
legend('True','EKF estimate','Start','Location','best'); grid on;

figure(2); set(gcf,'Name','EKF Fig 2 - States');
labels = {'x [m]','y [m]','vx [m/s]','vy [m/s]'};
for i = 1:4
    subplot(2,2,i);
    plot(t,X_true(i,:),'-','Color',c_true,'LineWidth',2); hold on;
    plot(t,X_est(i,:),'--','Color',c_est,'LineWidth',2);
    ylabel(labels{i}); legend('True','EKF'); grid on;
    if i>=3; xlabel('Time [s]'); end
end
sgtitle('EKF Fig 2 - State Estimates vs True States');

figure(3); set(gcf,'Name','EKF Fig 3 - Position Error');
plot(t,pos_err,'-','Color',c_est,'LineWidth',2);
xlabel('Time [s]'); ylabel('Position error [m]');
title('EKF Fig 3 - Estimation Error (should converge)'); grid on;

figure(4); set(gcf,'Name','EKF Fig 4 - Innovation');
subplot(2,1,1);
plot(t,innov(1,:),'-','Color',c_inn,'LineWidth',1.5); hold on; yline(0,'k--');
ylabel('r_u [px]'); title('EKF Fig 4 - Innovation r = z - h(X_{pred})'); grid on;
subplot(2,1,2);
plot(t,innov(2,:),'-','Color',c_inn,'LineWidth',1.5); hold on; yline(0,'k--');
xlabel('Time [s]'); ylabel('r_v [px]'); grid on;

figure(5); set(gcf,'Name','EKF Fig 5 - Covariance');
for i=1:n; P_d(i,:)=squeeze(P_arr(i,i,:))'; end
semilogy(t,P_d(1,:),'-','LineWidth',2); hold on;
semilogy(t,P_d(2,:),'--','LineWidth',2);
semilogy(t,P_d(3,:),':','LineWidth',2);
semilogy(t,P_d(4,:),'-.','LineWidth',2);
xlabel('Time [s]'); ylabel('Variance [log]');
title('EKF Fig 5 - Covariance (should decrease)');
legend('P_x','P_y','P_{vx}','P_{vy}','Location','best'); grid on;

fprintf('ekf.m complete. 5 figures generated.\n');