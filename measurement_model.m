% =========================================================
% measurement_model.m — Nonlinear Measurement Function h(X)
%                        and Measurement Jacobian H
%
% PURPOSE:
%   Replace the old constant C matrix with the real camera
%   projection model. The camera does not measure x,y directly
%   - it measures pixels u,v through perspective projection:
%
%       u = fx * (x / z)
%       v = fy * (y / z)
%
%   This is nonlinear because of the division by z.
%   So we cannot write y = C*X anymore. Instead: y = h(X).
%
%   For EKF, we need the Jacobian H = dh/dX (linearisation).
%
% HOW TO RUN:
%   Press F5 or type: measurement_model
%   (Self-contained — no other files required)
% =========================================================

clear; clc;

fprintf('=========================================\n');
fprintf('  MEASUREMENT MODEL — h(X) and Jacobian H\n');
fprintf('=========================================\n\n');

%% ---- Camera parameters ----
fx = 800;    % Focal length x [pixels]
fy = 800;    % Focal length y [pixels]
z  = 150;    % Depth: UAV altitude [metres] — constant for now

fprintf('Camera parameters: fx=%d px, fy=%d px, z=%d m\n\n', fx, fy, z);

%% ====================================================
%%  PART 1: NONLINEAR MEASUREMENT FUNCTION h(X)
%% ====================================================
fprintf('--- PART 1: h(X) ---\n\n');

% State X = [x, y, vx, vy]
% h(X) maps state to expected pixel measurement [u; v]
h = @(X) [ fx * (X(1) / z);
           fy * (X(2) / z) ];

fprintf('h(X) = [fx*x/z ; fy*y/z]\n\n');

X_test = [50; 30; 2; 1];
y_meas = h(X_test);

fprintf('Test: X = [%.0f, %.0f, %.0f, %.0f]\n', X_test(1),X_test(2),X_test(3),X_test(4));
fprintf('  h(X) = [u; v] = [%.2f; %.2f] pixels\n\n', y_meas(1), y_meas(2));

%% ====================================================
%%  PART 2: JACOBIAN H = dh/dX
%% ====================================================
fprintf('--- PART 2: Jacobian H ---\n\n');

% Partial derivatives:
%   du/dx = fx/z,  du/dy = 0,  du/dvx = 0,  du/dvy = 0
%   dv/dx = 0,     dv/dy = fy/z, dv/dvx = 0, dv/dvy = 0

H_func = @(X) [ fx/z   0    0   0;
                 0    fy/z   0   0 ];

H_test = H_func(X_test);

fprintf('H = [fx/z  0    0  0; 0  fy/z  0  0]\n\n');
fprintf('H evaluated at test state:\n');
disp(H_test);

fprintf('Compare with old constant C:\n');
C_old = [1 0 0 0; 0 1 0 0];
disp(C_old);
fprintf('When fx=fy=z, H reduces exactly to C.\n\n');

%% ====================================================
%%  PART 3: INNOVATION EXAMPLE
%% ====================================================
fprintf('--- PART 3: Innovation = Actual - Predicted ---\n\n');

X_true = [52; 31; 2.1; 0.9];
X_hat  = [50; 30; 2.0; 1.0];

z_actual    = h(X_true);
z_predicted = h(X_hat);
innovation  = z_actual - z_predicted;

fprintf('True state:      [%.1f, %.1f, %.1f, %.1f]\n', X_true(1),X_true(2),X_true(3),X_true(4));
fprintf('Estimated state:  [%.1f, %.1f, %.1f, %.1f]\n\n', X_hat(1),X_hat(2),X_hat(3),X_hat(4));
fprintf('Actual measurement    = [%.4f; %.4f] px\n', z_actual(1), z_actual(2));
fprintf('Predicted measurement = [%.4f; %.4f] px\n', z_predicted(1), z_predicted(2));
fprintf('Innovation r           = [%.4f; %.4f] px\n\n', innovation(1), innovation(2));

fprintf('measurement_model.m complete.\n');