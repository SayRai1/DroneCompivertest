function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
%MPC_RACING_CONTROLLER  Stable holonomic crab + smoothed guide + corner slow-down.
%  NO yaw, NO velocity damping (both destabilised - kept out).
%   - bearing_smooth : a low-pass "guide" direction, so a frame-to-frame detector
%                      flip at the elbow does not jerk the set-point.
%   - corner slow-down: when the guide points far off (a bend), shorten the
%                      look-ahead so the crab eases onto the new arm instead of
%                      orbiting the corner.
%  The drone follows the track by pure translation (nose held) -> cannot spin.
%#codegen

persistent bearing_smooth
if isempty(bearing_smooth); bearing_smooth = 0.0; end

x_err = double(x_err);   y_err = double(y_err);
psi   = double(psi);
x_est = double(x_est);   y_est = double(y_est);
takeoff_flag = double(takeoff_flag);

% ── Tuneable parameters ───────────────────────────────────────────────────
LOOKAHEAD = 0.10;   % straight set-point distance [m] (speed)
L_SLOW    = 0.04;   % set-point distance at a corner [m] (ease onto the new arm)
TH_SLOW   = deg2rad(25);   % rad ~34deg : guide bigger than this = a bend -> slow down
B_ALPHA   = 0.25;   % guide low-pass weight (small = steadier, absorbs flips)
ERR_MIN   = 2.4;    % px track-lost threshold
Z_TRACK   = -1.1;   % height [m NED]

err_mag = hypot(x_err, y_err);

if takeoff_flag && err_mag > ERR_MIN
    bearing_body = atan2(y_err, x_err);
    % smoothed "guide" direction (wrap-safe EMA) - absorbs detector flips
    d = atan2(sin(bearing_body - bearing_smooth), cos(bearing_body - bearing_smooth));
    bearing_smooth = bearing_smooth + B_ALPHA * d;

    % corner slow-down (no yaw): shorten look-ahead so the crab settles in
    if abs(bearing_smooth) > TH_SLOW
        v = L_SLOW;
    else
        v = LOOKAHEAD;
    end

    bearing_ned = psi + bearing_smooth;
    x_planned   = x_est + v * cos(bearing_ned);
    y_planned   = y_est + v * sin(bearing_ned);
    yaw_planned = psi;          % NO yaw - hold heading (cannot spin)
else
    x_planned      = x_est;
    y_planned      = y_est;
    yaw_planned    = psi;
    bearing_smooth = 0.0;
end
z_planned = Z_TRACK;

end
