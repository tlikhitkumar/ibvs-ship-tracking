% =========================================================
% observability_analysis.m — Observability Matrix, Rank, Condition Number
%
% PURPOSE:
%   Answer the question: "Can the camera measurements alone
%   reconstruct the ship's full motion state?"
%
%   This is done by building the Observability Matrix O:
%
%       O = [ C        ]   <- what the camera directly sees
%           [ C*A      ]   <- what can be inferred 1 step ahead
%           [ C*A^2    ]   <- what can be inferred 2 steps ahead
%           [  ...     ]
%           [ C*A^(n-1)]
%
%   Then checking:
%       rank(O) == n  →  FULLY OBSERVABLE (all states recoverable)
%       rank(O) <  n  →  NOT FULLY OBSERVABLE
%
%   Also checking:
%       cond(O)        →  small = well-conditioned = easy to estimate
%                         large = ill-conditioned  = hard to estimate
%
% HOW TO RUN:
%   Press F5 or type: observability_analysis
%   (No inputs needed — everything is defined inside)
% =========================================================

clear; clc;

fprintf('=========================================\n');
fprintf('  OBSERVABILITY ANALYSIS\n');
fprintf('=========================================\n\n');

dt = 0.5;   % timestep (needed to build A)

%% ====================================================
%%  PART 1: CONSTANT VELOCITY (CV) MODEL
%% ====================================================
fprintf('=========================================\n');
fprintf('  PART 1: CV Model  (n = 4 states)\n');
fprintf('  State: [x, y, vx, vy]\n');
fprintf('=========================================\n\n');

%% ---- Step 1: State Transition Matrix A ----
% X(k+1) = A * X(k)
CV_A = [1  0  dt  0;
        0  1   0  dt;
        0  0   1   0;
        0  0   0   1];

fprintf('STEP 1 — State Transition Matrix A (4x4):\n');
disp(CV_A);
fprintf('  Interpretation:\n');
fprintf('  Row 1: x(k+1) = x(k) + vx*dt\n');
fprintf('  Row 2: y(k+1) = y(k) + vy*dt\n');
fprintf('  Row 3: vx stays constant\n');
fprintf('  Row 4: vy stays constant\n\n');

%% ---- Step 2: Measurement Matrix C ----
% Camera measures only [x, y] — NOT velocity directly
% y_measured = C * X
%
%   [x_meas]   [1 0 0 0] [x ]
%   [y_meas] = [0 1 0 0] [y ]
%                        [vx]
%                        [vy]
CV_C = [1 0 0 0;
        0 1 0 0];

fprintf('STEP 2 — Measurement Matrix C (2x4):\n');
disp(CV_C);
fprintf('  Interpretation:\n');
fprintf('  Row 1: measures x directly\n');
fprintf('  Row 2: measures y directly\n');
fprintf('  Velocity (vx, vy) is NOT directly measured — columns 3,4 are zero\n\n');

%% ---- Step 3: Build Observability Matrix O ----
% For n=4 states, we need 4 block rows: C, CA, CA^2, CA^3
n_cv = 4;

fprintf('STEP 3 — Building Observability Matrix O:\n\n');

CV_CA0 = CV_C;                % C        (what we measure directly)
CV_CA1 = CV_C * CV_A;         % C*A      (1 step propagated)
CV_CA2 = CV_C * (CV_A^2);     % C*A^2    (2 steps propagated)
CV_CA3 = CV_C * (CV_A^3);     % C*A^3    (3 steps propagated)

fprintf('  C       (direct measurement):\n'); disp(CV_CA0);
fprintf('  C*A     (1-step inference):\n');   disp(CV_CA1);
fprintf('  C*A^2   (2-step inference):\n');   disp(CV_CA2);
fprintf('  C*A^3   (3-step inference):\n');   disp(CV_CA3);

% Stack into one matrix
CV_O = [CV_CA0;
        CV_CA1;
        CV_CA2;
        CV_CA3];

fprintf('  Full Observability Matrix O (8x4) = [C; CA; CA^2; CA^3]:\n');
disp(CV_O);

%% ---- Step 4: Rank and Condition Number ----
CV_rank = rank(CV_O);
CV_cond = cond(CV_O);

fprintf('STEP 4 — Observability Test:\n');
fprintf('  n (number of states)  = %d\n', n_cv);
fprintf('  rank(O)               = %d\n', CV_rank);
fprintf('  cond(O)               = %.4e\n\n', CV_cond);

if CV_rank == n_cv
    fprintf('  RESULT: FULLY OBSERVABLE\n');
    fprintf('  All 4 states [x, y, vx, vy] can be reconstructed\n');
    fprintf('  from camera measurements of [x, y] alone.\n');
    fprintf('  Velocity is hidden but inferable from position changes.\n\n');
else
    fprintf('  RESULT: NOT FULLY OBSERVABLE\n');
    fprintf('  Only %d of %d states are recoverable.\n\n', CV_rank, n_cv);
end

%% ====================================================
%%  PART 2: CONSTANT ACCELERATION (CA) MODEL
%% ====================================================
fprintf('=========================================\n');
fprintf('  PART 2: CA Model  (n = 6 states)\n');
fprintf('  State: [x, y, vx, vy, ax, ay]\n');
fprintf('=========================================\n\n');

%% ---- Step 1: State Transition Matrix A ----
CA_A = [1  0  dt   0   0.5*dt^2    0      ;
        0  1   0  dt       0    0.5*dt^2  ;
        0  0   1   0      dt       0      ;
        0  0   0   1       0      dt      ;
        0  0   0   0       1       0      ;
        0  0   0   0       0       1      ];

fprintf('STEP 1 — State Transition Matrix A (6x6):\n');
disp(CA_A);

%% ---- Step 2: Measurement Matrix C ----
% Same idea: camera still only measures [x, y]
% But now state has 6 elements
CA_C = [1 0 0 0 0 0;
        0 1 0 0 0 0];

fprintf('STEP 2 — Measurement Matrix C (2x6):\n');
disp(CA_C);
fprintf('  Still measures only x and y.\n');
fprintf('  Velocity (vx,vy) and acceleration (ax,ay) NOT directly measured.\n\n');

%% ---- Step 3: Build Observability Matrix O ----
% For n=6 states, we need 6 block rows: C, CA, ..., CA^5
n_ca = 6;

fprintf('STEP 3 — Building Observability Matrix O:\n\n');

CA_O = zeros(2*n_ca, n_ca);
for i = 0:n_ca-1
    rows = (2*i+1):(2*i+2);
    CA_O(rows, :) = CA_C * (CA_A^i);
    fprintf('  C*A^%d:\n', i);
    disp(CA_C * (CA_A^i));
end

fprintf('  Full Observability Matrix O (12x6):\n');
disp(CA_O);

%% ---- Step 4: Rank and Condition Number ----
CA_rank = rank(CA_O);
CA_cond = cond(CA_O);

fprintf('STEP 4 — Observability Test:\n');
fprintf('  n (number of states)  = %d\n', n_ca);
fprintf('  rank(O)               = %d\n', CA_rank);
fprintf('  cond(O)               = %.4e\n\n', CA_cond);

if CA_rank == n_ca
    fprintf('  RESULT: FULLY OBSERVABLE\n');
    fprintf('  All 6 states [x, y, vx, vy, ax, ay] can be reconstructed.\n');
    fprintf('  But note: cond(O) is larger than CV model.\n');
    fprintf('  This means acceleration is harder to estimate accurately.\n\n');
else
    fprintf('  RESULT: NOT FULLY OBSERVABLE\n');
    fprintf('  Only %d of %d states are recoverable.\n\n', CA_rank, n_ca);
end

%% ====================================================
%%  PART 3: COMPARISON SUMMARY
%% ====================================================
fprintf('=========================================\n');
fprintf('  COMPARISON SUMMARY\n');
fprintf('=========================================\n\n');
fprintf('  Model   States   rank(O)   Full Obs?   cond(O)\n');
fprintf('  -----   ------   -------   ---------   -------\n');
fprintf('  CV        4        %d         %s      %.3e\n', ...
    CV_rank, yesno(CV_rank==n_cv), CV_cond);
fprintf('  CA        6        %d         %s      %.3e\n\n', ...
    CA_rank, yesno(CA_rank==n_ca), CA_cond);

fprintf('  KEY INSIGHT:\n');
fprintf('  Both models are fully observable — the camera\n');
fprintf('  measurements of [x,y] are enough to recover velocity\n');
fprintf('  (and acceleration in CA). However, CA has a larger\n');
fprintf('  condition number — accelerations are harder to estimate.\n\n');

fprintf('observability_analysis.m complete.\n');

%% ---- Helper function ----
function s = yesno(val)
    if val
        s = 'YES';
    else
        s = 'NO ';
    end
end