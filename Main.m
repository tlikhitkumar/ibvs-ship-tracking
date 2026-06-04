% =========================================================
% main.m — Master Script (Runs Everything + All Plots)
%
% PURPOSE:
%   Tie all parts together and generate the final results.
%   This file does NOT redefine anything — it runs the other
%   files in order and uses their outputs for plotting.
%
%   Run order:
%     1. config.m                  → loads all parameters
%     2. ship trajectory (inline)  → CV and CA state histories
%     3. camera projection (inline)→ image measurements
%     4. observability (inline)    → rank and condition number
%     5. 7 figures
%
% HOW TO RUN:
%   Press F5 or type: main
% =========================================================

clear; clc; close all;

fprintf('============================================\n');
fprintf('  IBVS SHIP TRACKING — FULL SIMULATION\n');
fprintf('============================================\n\n');

%% ---- STEP 1: Load parameters ----
Config;

%% ---- STEP 2: Ship Motion (same logic as ship_motion.m) ----
fprintf('\n[1/4] Running ship motion models...\n');

CV_A = [1  0  dt  0;
        0  1   0  dt;
        0  0   1   0;
        0  0   0   1];

CA_A = [1  0  dt   0   0.5*dt^2    0      ;
        0  1   0  dt       0    0.5*dt^2  ;
        0  0   1   0      dt       0      ;
        0  0   0   1       0      dt      ;
        0  0   0   0       1       0      ;
        0  0   0   0       0       1      ];

CV_X = zeros(4, N_steps);  CV_X(:,1) = CV_X0;
CA_X = zeros(6, N_steps);  CA_X(:,1) = CA_X0;

for k = 1:N_steps-1
    CV_X(:,k+1) = CV_A * CV_X(:,k);
    CA_X(:,k+1) = CA_A * CA_X(:,k);
end
fprintf('    CV and CA trajectories generated.\n');

%% ---- STEP 3: Camera Projection (same logic as camera_projection.m) ----
fprintf('\n[2/4] Running camera projection...\n');

R = [1  0  0;
     0 -1  0;
     0  0 -1];

CV_u = zeros(N_features, N_steps);  CV_v = zeros(N_features, N_steps);
CA_u = zeros(N_features, N_steps);  CA_v = zeros(N_features, N_steps);

for k = 1:N_steps
    uav_x = UAV_x0 + UAV_vx * (k-1) * dt;
    uav_y = UAV_y0 + UAV_vy * (k-1) * dt;
    d = [uav_x; uav_y; UAV_alt];

    for f = 1:N_features
        % CV projection
        Pw = [CV_X(1,k)+ship_features(1,f);
              CV_X(2,k)+ship_features(2,f);
              ship_features(3,f)];
        Pc = R*(Pw-d);
        if Pc(3)>0
            CV_u(f,k)=lambda*(Pc(1)/Pc(3));
            CV_v(f,k)=lambda*(Pc(2)/Pc(3));
        end

        % CA projection
        Pw = [CA_X(1,k)+ship_features(1,f);
              CA_X(2,k)+ship_features(2,f);
              ship_features(3,f)];
        Pc = R*(Pw-d);
        if Pc(3)>0
            CA_u(f,k)=lambda*(Pc(1)/Pc(3));
            CA_v(f,k)=lambda*(Pc(2)/Pc(3));
        end
    end
end
fprintf('    Image coordinates generated.\n');

%% ---- STEP 4: Observability (same logic as observability_analysis.m) ----
fprintf('\n[3/4] Running observability analysis...\n');

% Measurement matrices
CV_C = [1 0 0 0; 0 1 0 0];
CA_C = [1 0 0 0 0 0; 0 1 0 0 0 0];

% Build O matrices
n_cv = 4;
CV_O = cell2mat(arrayfun(@(i) CV_C*(CV_A^i), 0:n_cv-1, 'UniformOutput',false)');

n_ca = 6;
CA_O = cell2mat(arrayfun(@(i) CA_C*(CA_A^i), 0:n_ca-1, 'UniformOutput',false)');

CV_rank = rank(CV_O);  CV_cond = cond(CV_O);
CA_rank = rank(CA_O);  CA_cond = cond(CA_O);

% Store as time vectors (LTI — constant across time)
CV_rank_t = CV_rank * ones(1, N_steps);
CA_rank_t = CA_rank * ones(1, N_steps);
CV_cond_t = CV_cond * ones(1, N_steps);
CA_cond_t = CA_cond * ones(1, N_steps);

fprintf('    CV: rank=%d/4, cond=%.3e\n', CV_rank, CV_cond);
fprintf('    CA: rank=%d/6, cond=%.3e\n', CA_rank, CA_cond);

%% ====================================================
%%  PLOTS
%% ====================================================
fprintf('\n[4/4] Generating plots...\n');

col_cv = [0.15 0.45 0.80];
col_ca = [0.85 0.33 0.10];

% -------------------------------------------------------
% Figure 1: Ship Trajectories
% -------------------------------------------------------
figure(1); set(gcf,'Name','Fig 1 — Ship Trajectories');
plot(CV_X(1,:), CV_X(2,:), '-',  'Color',col_cv, 'LineWidth',2); hold on;
plot(CA_X(1,:), CA_X(2,:), '--', 'Color',col_ca, 'LineWidth',2);
plot(CV_X(1,1),  CV_X(2,1),  'ko','MarkerSize',8,'MarkerFaceColor','k');
plot(CV_X(1,end),CV_X(2,end),'ks','MarkerSize',8,'MarkerFaceColor',col_cv);
plot(CA_X(1,end),CA_X(2,end),'kd','MarkerSize',8,'MarkerFaceColor',col_ca);
xlabel('X [m]'); ylabel('Y [m]');
title('Fig 1 — Ship Trajectories (World Frame)');
legend('CV (straight)','CA (curved)','Start','CV End','CA End','Location','best');
grid on;

% -------------------------------------------------------
% Figure 2: Image Feature Trajectories
% -------------------------------------------------------
figure(2); set(gcf,'Name','Fig 2 — Image Feature Trajectories');
subplot(1,2,1); hold on;
for f=1:N_features
    plot(CV_u(f,:),CV_v(f,:),'-','LineWidth',1.5);
    plot(CV_u(f,1),CV_v(f,1),'ko','MarkerSize',5);
end
xlabel('u [px]'); ylabel('v [px]');
title('CV — Image Features'); grid on;
legend(arrayfun(@(x)sprintf('F%d',x),1:N_features,'UniformOutput',false));

subplot(1,2,2); hold on;
for f=1:N_features
    plot(CA_u(f,:),CA_v(f,:),'--','LineWidth',1.5);
    plot(CA_u(f,1),CA_v(f,1),'ko','MarkerSize',5);
end
xlabel('u [px]'); ylabel('v [px]');
title('CA — Image Features'); grid on;
legend(arrayfun(@(x)sprintf('F%d',x),1:N_features,'UniformOutput',false));

% -------------------------------------------------------
% Figure 3: Position vs Time
% -------------------------------------------------------
figure(3); set(gcf,'Name','Fig 3 — Position vs Time');
subplot(2,1,1);
plot(t,CV_X(1,:),'-','Color',col_cv,'LineWidth',2); hold on;
plot(t,CA_X(1,:),'--','Color',col_ca,'LineWidth',2);
ylabel('X [m]'); title('Fig 3 — Position vs Time');
legend('CV','CA'); grid on;
subplot(2,1,2);
plot(t,CV_X(2,:),'-','Color',col_cv,'LineWidth',2); hold on;
plot(t,CA_X(2,:),'--','Color',col_ca,'LineWidth',2);
xlabel('Time [s]'); ylabel('Y [m]');
legend('CV','CA'); grid on;

% -------------------------------------------------------
% Figure 4: Velocity vs Time
% -------------------------------------------------------
figure(4); set(gcf,'Name','Fig 4 — Velocity vs Time');
subplot(2,1,1);
plot(t,CV_X(3,:),'-','Color',col_cv,'LineWidth',2); hold on;
plot(t,CA_X(3,:),'--','Color',col_ca,'LineWidth',2);
ylabel('Vx [m/s]'); title('Fig 4 — Velocity vs Time');
legend('CV (const)','CA (varying)'); grid on;
subplot(2,1,2);
plot(t,CV_X(4,:),'-','Color',col_cv,'LineWidth',2); hold on;
plot(t,CA_X(4,:),'--','Color',col_ca,'LineWidth',2);
xlabel('Time [s]'); ylabel('Vy [m/s]');
legend('CV (const)','CA (varying)'); grid on;

% -------------------------------------------------------
% Figure 5: Observability Rank vs Time
% -------------------------------------------------------
figure(5); set(gcf,'Name','Fig 5 — Rank vs Time (KEY RESULT)');
plot(t, CV_rank_t, '-',  'Color',col_cv, 'LineWidth',2.5); hold on;
plot(t, CA_rank_t, '--', 'Color',col_ca, 'LineWidth',2.5);
yline(4,':','CV Full (n=4)','Color',col_cv,'LabelHorizontalAlignment','left','LineWidth',1.5);
yline(6,':','CA Full (n=6)','Color',col_ca,'LabelHorizontalAlignment','left','LineWidth',1.5);
xlabel('Time [s]'); ylabel('rank(O)');
title('Fig 5 — Observability Rank vs Time  [KEY RESULT]');
legend('CV Model (n=4)','CA Model (n=6)','Location','best');
ylim([0 8]); yticks(0:1:8); grid on;

% -------------------------------------------------------
% Figure 6: Condition Number vs Time
% -------------------------------------------------------
figure(6); set(gcf,'Name','Fig 6 — Condition Number vs Time');
semilogy(t, CV_cond_t, '-',  'Color',col_cv, 'LineWidth',2); hold on;
semilogy(t, CA_cond_t, '--', 'Color',col_ca, 'LineWidth',2);
xlabel('Time [s]'); ylabel('cond(O)  [log scale]');
title('Fig 6 — Condition Number vs Time');
legend('CV Model','CA Model','Location','best'); grid on;

% -------------------------------------------------------
% Figure 7: Side-by-side comparison
% -------------------------------------------------------
figure(7); set(gcf,'Name','Fig 7 — CV vs CA Comparison');
set(gcf,'Position',[100 100 900 380]);

subplot(1,2,1);
b = bar([CV_rank, CA_rank], 'FaceColor','flat');
b.CData = [col_cv; col_ca];
set(gca,'XTickLabel',{'CV (n=4)','CA (n=6)'});
hold on;
plot([0.5 1.5],[4 4],'b:','LineWidth',1.5);
plot([1.5 2.5],[6 6],'r:','LineWidth',1.5);
ylabel('rank(O)'); title('Rank Comparison');
ylim([0 8]); yticks(0:1:8); grid on;
legend('Achieved Rank','Full Observable Target','Location','best');

subplot(1,2,2);
b2 = bar([log10(CV_cond), log10(CA_cond)], 'FaceColor','flat');
b2.CData = [col_cv; col_ca];
set(gca,'XTickLabel',{'CV (n=4)','CA (n=6)'});
ylabel('log_{10}(cond(O))');
title('Condition Number (log scale)');
grid on;

%% ---- Final Summary ----
fprintf('\n============================================\n');
fprintf('  FINAL RESULTS\n');
fprintf('============================================\n');
fprintf('  CV: rank=%d/4  |  cond=%.3e  |  Observable: %s\n', ...
    CV_rank, CV_cond, yesno(CV_rank==n_cv));
fprintf('  CA: rank=%d/6  |  cond=%.3e  |  Observable: %s\n', ...
    CA_rank, CA_cond, yesno(CA_rank==n_ca));
fprintf('  7 figures generated.\n');
fprintf('============================================\n');

function s = yesno(val)
    if val; s='YES'; else; s='NO'; end
end