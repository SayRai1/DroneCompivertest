function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
%MPC_RACING_CONTROLLER  Racing-line POSITION planner (replaces Pure Pursuit).
%  Outputs an absolute NED position reference pos_ref = [x_planned; y_planned;
%  z_planned] (position mode, controlModePosVsOrient = 1) plus an optional
%  heading command yaw_planned.
%
%  ── WHY THE DRONE USED TO STOP AT CORNERS ────────────────────────────────
%   The previous law placed the set-point proportional to the pixel error:
%       x_planned = x_est + K_TRACK*x_err
%   On a corner the drone centres on the track so x_err,y_err -> 0, hence the
%   set-point collapses onto the drone (x_planned ~= x_est) and it STOPS dead
%   in the corner.  This version instead aims a FIXED look-ahead DISTANCE
%   along the track bearing, so there is always forward progress, and it
%   shortens that distance in sharp bends to brake into the corner.
%
%  ── RACING BEHAVIOUR ──────────────────────────────────────────────────────
%     straight (small |y_err|)   -> lookahead = L_MAX  -> FAST
%     sharp bend (large |y_err|) -> lookahead = L_MIN  -> BRAKE
%   The set-point is placed at that distance along the bearing to the track
%   look-ahead point, so the drone flows through corners instead of stopping.
%
%  ── INPUTS ────────────────────────────────────────────────────────────────
%   x_err,y_err  : longitudinal / lateral track pixel error (decision_making)
%   phi,theta    : roll/pitch [rad]            (unused - interface only)
%   psi          : yaw [rad]   (used for the body->NED look-ahead rotation)
%   vx,vy,vz,p,q,r : body vel / rates          (unused - interface only)
%   takeoff_flag : wired to [flag_activate_PP] - 1 = path planning active
%                  (track), 0 = still taking off (hold)
%   x_est,y_est  : current NED position estimate [m]
%
%  ── OUTPUTS ───────────────────────────────────────────────────────────────
%   x_planned,y_planned,z_planned : NED position reference [m]  (-> pos_ref)
%   yaw_planned                   : heading reference [rad]
%                  (currently terminated; route to orient_ref(3) to enable
%                   "nose into the corner".  YAW_GAIN = 0 keeps heading hold.)

%#codegen
%#codegen

% เก็บ bearing เฟรมก่อนหน้า (เอาไว้คำนวณว่าทิศเปลี่ยนเร็วแค่ไหน = ความโค้ง)
persistent prev_bearing
if isempty(prev_bearing); prev_bearing = 0.0; end

% ── Force all used inputs to double (state/vision buses are single) ───────
x_err = double(x_err);   y_err = double(y_err);
psi   = double(psi);
x_est = double(x_est);   y_est = double(y_est);
takeoff_flag = double(takeoff_flag);

% ── Tuneable parameters ───────────────────────────────────────────────────
% ── Tuneable parameters ───────────────────────────────────────────────────
L_MAX       = 0.19;   % look-ahead ทางตรง [m]   <<< SLOW MODE (เดิม 0.55) ช้าๆ ตามเส้นก่อน
L_MIN       = 0.11;   % look-ahead/ความเร็วขั้นต่ำในโค้ง [m] (0.12->0.18 = โค้งเร็วขึ้น; =L_MAX คือไม่ชะลอเลย)
BEARING_REF = 0.55;   % rad ~23°  จุดที่ "มุมโค้งปัจจุบัน" เบรกเต็ม (เล็ก = ไวต่อโค้ง)
CURV_GAIN   = 0.2;    % น้ำหนัก "เบรกก่อนถึงโค้ง" (ช้าอยู่แล้ว เบรกเบาๆ พอ)
ERR_MIN     = 2.5;    % px track-lost threshold
Z_TRACK     = -1.1;   % ความสูง [m NED]
YAW_GAIN    = 1.0;    % 0 = hold heading

err_mag = hypot(x_err, y_err);

if takeoff_flag && err_mag > ERR_MIN
    % ── Bearing to the look-ahead track point (image/body frame) ──────────
    bearing_body = atan2(y_err, x_err);


    % ── Speed / brake profile ─────────────────────────────────────────────
        % ── Curvature-anticipatory brake ──────────────────────────────────────
    % d_bear = ทิศเปลี่ยนไปเท่าไรจากเฟรมก่อน (atan2 กันค่ากระโดด ±pi)
    d_bear       = atan2(sin(bearing_body - prev_bearing), cos(bearing_body - prev_bearing));
    prev_bearing = bearing_body;
    % corner = มุมโค้งตอนนี้ + อัตราที่โค้งกำลังหักขึ้น (= เบรกก่อนถึงโค้ง)
    corner    = min( abs(bearing_body)/BEARING_REF + CURV_GAIN*abs(d_bear), 1.0 );
    lookahead = L_MIN + (L_MAX - L_MIN) * (1.0 - corner);

    % ── Place the set-point a look-ahead distance ahead, along the track ──
    bearing_ned = psi + bearing_body;                    % body -> NED (psi = heading)
    x_planned   = x_est + lookahead * cos(bearing_ned);
    y_planned   = y_est + lookahead * sin(bearing_ned);

    % ── Heading: aim the nose toward the look-ahead point ─────────────────
    yaw_planned = psi + YAW_GAIN * bearing_body;
else
    % Taking off, or track lost -> hold current position & heading
    x_planned   = x_est;
    y_planned   = y_est;
    yaw_planned = psi;
end
z_planned = Z_TRACK;

end
