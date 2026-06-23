function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
%MPC_RACING_CONTROLLER  Line-following planner with a TRANSLATE/ROTATE supervisor.
%  Position mode (controlModePosVsOrient = 1): outputs an absolute NED
%  position reference [x_planned; y_planned; z_planned] (-> pos_ref) plus a
%  heading command yaw_planned (-> orient_ref Parrot-yaw = element 1, after the
%  Mux fix in Path Planning).
%
%  WHY (old law was one overloaded controller -> circled at corners):
%    e_psi = filtered bearing to the look-ahead point  (= heading error)
%    SUPERVISOR (hysteresis):
%        |e_psi| > TH_HI  -> ROTATE   : hover & turn the nose onto the line
%        |e_psi| < TH_LO  -> TRANSLATE: fly along the line
%    v_along = V_MAX*(1 - |e_psi|/TH_FULL)  (slow into bends; 0 while rotating)
%    yaw     = psi + YAW_GAIN*e_psi          (always aim nose at the line)
%    pos_ref = pursuit point v_along ahead along the (proven) bearing
%
%  INPUTS : x_err,y_err longitudinal/lateral track pixel error; psi yaw [rad];
%           takeoff_flag (1=track active); x_est,y_est NED position [m].
%           (phi,theta,vx..r unused - interface only)
%  OUTPUTS: x_planned,y_planned,z_planned NED ref [m]; yaw_planned [rad].

%#codegen

% ── persistent filter / supervisor state ──────────────────────────────────
persistent bearing_smooth   % EMA-filtered heading error [rad]
persistent mode             % 0 = TRANSLATE, 1 = ROTATE-first
persistent rot_count        % consecutive frames heading error has exceeded TH_HI
if isempty(bearing_smooth); bearing_smooth = 0.0; end
if isempty(mode);           mode           = 0.0; end
if isempty(rot_count);      rot_count      = 0.0; end

% ── force used inputs to double (state/vision buses are single) ───────────
x_err = double(x_err);   y_err = double(y_err);
psi   = double(psi);
vx    = double(vx);      vy    = double(vy);
x_est = double(x_est);   y_est = double(y_est);
takeoff_flag = double(takeoff_flag);

% ── Tuneable parameters ───────────────────────────────────────────────────
V_MAX     = 0.19;   % look-ahead dist / forward speed on a straight [m]
YAW_GAIN  = 0.6;    % nose-to-line gain (yaw_ref = psi + YAW_GAIN*e_psi)
YAW_ALPHA = 0.30;   % EMA weight on heading error (small = smoother / laggier)
TH_HI     = 0.87;   % rad ~50 deg -> rotate ONLY on a sharp turn (was 35: rotated too eagerly)
TH_LO     = 0.35;   % rad ~20 deg -> back to TRANSLATE (hysteresis)
TH_FULL   = 1.40;   % rad ~80 deg -> keep more straight speed; slow less into bends
ROT_DWELL = 8;      % must exceed TH_HI for this many frames -> CONFIRMED turn (prefer straight)
K_DAMP    = 0.30;   % velocity damping -> more braking = less overshoot (was 0.15)
ERR_MIN   = 2.5;    % px  track-lost threshold
Z_TRACK   = -1.1;   % height [m NED]

err_mag = hypot(x_err, y_err);

if takeoff_flag && err_mag > ERR_MIN
    % heading error = bearing to the look-ahead point (body frame), filtered
    bearing_body   = atan2(y_err, x_err);
    bearing_smooth = (1.0 - YAW_ALPHA)*bearing_smooth + YAW_ALPHA*bearing_body;
    e_abs          = abs(bearing_smooth);

    % ── SUPERVISOR: prefer TRANSLATE; rotate only on a CONFIRMED sharp turn ─
    % count consecutive frames heading error stays large (= a real bend, not
    % vision noise) before committing to ROTATE -> "wait for the right moment".
    if e_abs > TH_HI
        rot_count = rot_count + 1.0;
    else
        rot_count = 0.0;
    end
    if rot_count >= ROT_DWELL
        mode = 1.0;          % sustained sharp misalignment -> rotate the nose
    elseif e_abs < TH_LO
        mode = 0.0;          % aligned -> translate (default = go straight)
        rot_count = 0.0;
    end                      % in between: keep previous mode

    % ── forward speed: 0 while rotating, else slow with misalignment ──────
    if mode > 0.5
        v_along = 0.0;
    else
        v_along = V_MAX * max(0.0, 1.0 - e_abs/TH_FULL);
    end

    % ── yaw: aim the nose toward the line (wrapped to [-pi, pi]) ───────────
    yaw_raw     = psi + YAW_GAIN * bearing_smooth;
    yaw_planned = atan2(sin(yaw_raw), cos(yaw_raw));

    % ── position set-point: pursuit point v_along ahead, minus velocity
    %     damping (aims slightly behind momentum -> brakes -> cuts overshoot) ─
    vN = vx*cos(psi) - vy*sin(psi);     % body velocity -> NED
    vE = vx*sin(psi) + vy*cos(psi);
    bearing_ned = psi + bearing_body;
    x_planned   = x_est + v_along*cos(bearing_ned) - K_DAMP*vN;
    y_planned   = y_est + v_along*sin(bearing_ned) - K_DAMP*vE;
else
    % taking off / track lost -> hold pose, reset state (no spike on re-lock)
    x_planned      = x_est;
    y_planned      = y_est;
    yaw_planned    = psi;
    bearing_smooth = 0.0;
    mode           = 0.0;
    rot_count      = 0.0;
end
z_planned = Z_TRACK;

end
