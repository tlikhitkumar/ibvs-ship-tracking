# IBVS Ship Tracking — Observability Analysis

MATLAB simulation for Image-Based Visual Servoing (IBVS) applied to ship tracking using a UAV-mounted camera. The project studies whether camera measurements can reconstruct the ship's full motion state, using both linear (constant C) and nonlinear (Jacobian-based) observability analysis, and an Extended Kalman Filter for state estimation.

## How to run — Part 1: Linear observability (original files)

These files use a simplified linear measurement model where the camera is assumed to directly measure world position (x, y).

Run in this order:

1. `config.m` — defines all simulation parameters (timestep, UAV altitude, focal length, ship initial state). Run this first; nothing else works without it.
2. `ship_motion.m` — simulates the ship's motion using Constant Velocity (CV) and Constant Acceleration (CA) models.
3. `camera_projection.m` — projects the 3D ship position into 2D camera pixel coordinates.
4. `observability_analysis.m` — builds the observability matrix O, computes rank and condition number, for both CV and CA models.
5. `main.m` — runs everything above in sequence and generates all plots. **Run this one file to see the full Part 1 pipeline.**

## How to run — Part 2: EKF and nonlinear observability (new files)

These files replace the simplified linear camera model with the real nonlinear pinhole projection (`u = fx*x/z`, `v = fy*y/z`), and use an Extended Kalman Filter to estimate ship state from noisy pixel measurements. Each file is self-contained and can be run independently with F5.

Recommended run order:

1. `measurement_model.m` — defines the nonlinear measurement function h(X) and its Jacobian H. Explains the core math change from the old constant C matrix.
2. `ekf.m` — runs the Extended Kalman Filter: predicts ship state, compares predicted measurement to noisy camera measurement, corrects the estimate. Produces 5 figures showing tracking accuracy.
3. `nonlinear_observability.m` — compares the old LTI approach (constant C, flat rank over time) against the new LTV approach (Jacobian H, rank rebuilt at every timestep). Produces 3 figures.
4. `main_ekf.m` — runs the full Part 2 pipeline (EKF + LTI vs LTV observability) in one script. **Run this one file to see the full Part 2 pipeline.** Produces 12 figures.

## Quick summary: which file to run for what

| Goal | File to run |
|---|---|
| See the original linear simulation, all plots | `main.m` |
| See EKF tracking + nonlinear observability, all plots | `main_ekf.m` |
| Understand the math behind h(X) and the Jacobian H | `measurement_model.m` |
| Understand EKF predict/update steps in isolation | `ekf.m` |
| Understand how rank(O) differs between constant C and time-varying H | `nonlinear_observability.m` |
