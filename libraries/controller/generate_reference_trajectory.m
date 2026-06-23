function ref_traj = generate_reference_trajectory(x_err, y_err, vx_body, vy_body, N, Ts)
%GENERATE_REFERENCE_TRAJECTORY  Extrapolate a horizon of N reference points
%  from a single look-ahead pixel error (x_err = longitudinal, y_err = lateral)
%  and the current body-frame velocities.
%
%  Inputs
%    x_err   – longitudinal pixel error (+ve = track centre is ahead)  [scalar]
%    y_err   – lateral pixel error       (+ve = track centre is right)  [scalar]
%    vx_body – body-frame forward  velocity  [m/s]
%    vy_body – body-frame lateral  velocity  [m/s]
%    N       – prediction horizon length
%    Ts      – step size of the VISION sample (VTs = 0.2 s typical)
%
%  Output
%    ref_traj – (N x 2) matrix  [x_err_k, y_err_k] for k = 1 … N
%
%  Strategy:
%    We assume the drone is currently travelling with (vx, vy) in pixel/s
%    (converted from m/s via PX_PER_M).  Under zero control effort the
%    errors evolve as a decelerating first-order system towards the
%    look-ahead target.  We interpolate a smooth exponential decay from
%    the current error to zero over the horizon.  This gives the MPC a
%    "gentle apex approach" target rather than the raw step reference that
%    Pure Pursuit uses.

% ── Physical pixel scale ───────────────────────────────────────────────────
% At 1 m flight height, FOV = 2*(pi/2.9) rad, image width = 160 px.
%   half_fov = pi/2.9 ≈ 1.0840 rad
%   image half-width = 80 px
%   tan(half_fov)*1m ≈ 1.808 m  →  80 px / 1.808 m ≈ 44.25 px/m
PX_PER_M = 44.25;

% ── Convert velocity to pixel/s ────────────────────────────────────────────
vx_px = vx_body * PX_PER_M;   % px/s, forward (x) direction
vy_px = vy_body * PX_PER_M;   % px/s, lateral (y) direction

% ── Time vector ───────────────────────────────────────────────────────────
t = (1:N)' * Ts;               % N×1

% ── Time constant: error should converge in ~half the horizon ─────────────
tau = N * Ts * 0.5;

% ── Exponential decay towards zero error (the track centre) ───────────────
%  x_ref(t) = x_err * exp(-t/tau)   (aim to eliminate longitudinal error)
%  y_ref(t) = y_err * exp(-t/tau)   (aim to centre laterally)
x_ref = x_err .* exp(-t ./ tau);
y_ref = y_err .* exp(-t ./ tau);

% ── Inject a velocity-feed-forward bias for apex cutting ──────────────────
%  If we are heading into a corner (large |y_err| and vx positive), we
%  shift the reference so the drone starts rolling before reaching the
%  apex.  This implements an "early apex" strategy.
%
%  Bias  = v_lateral * t * 0.35   (damped look-ahead)
lateral_bias = vy_px .* t .* 0.35 .* exp(-t ./ tau);
y_ref = y_ref + lateral_bias;

% ── Pack output ──────────────────────────────────────────────────────────
ref_traj = [x_ref, y_ref];   % N × 2

end
