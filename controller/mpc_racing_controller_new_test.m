function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
% MPC_RACING_CONTROLLER V6 — FIXED REVERSE-AT-CORNER BUG
%
%   Root cause: anti-reverse guard FLIPS bearing at sharp corners
%   because the new leg IS behind the drone — the flip fights the
%   correct correction.
%
%   Fix: DISABLE anti-reverse when corner is detected. At corners
%   trust raw vision bearing 100%, brake hard, place waypoint
%   directly toward the detected line.
%
%#codegen

% ========================  PERSISTENT STATES  ============================
persistent bearing_smooth prev_bearing prev_x prev_y kappa_smooth ...
           bear_rate_smooth corner_cnt
if isempty(bearing_smooth)
    bearing_smooth    = 0.0;
    prev_bearing      = 0.0;
    prev_x            = 0.0;
    prev_y            = 0.0;
    kappa_smooth      = 0.0;
    bear_rate_smooth  = 0.0;
    corner_cnt        = 0;       % counter: กี่ step ที่อยู่ใน corner mode
end

% ========================  CAST TO DOUBLE  ===============================
x_err = double(x_err);  y_err = double(y_err);
phi   = double(phi);     theta = double(theta);  psi = double(psi);
vx    = double(vx);      vy    = double(vy);     vz  = double(vz);
p     = double(p);       q     = double(q);      r   = double(r);
x_est = double(x_est);   y_est = double(y_est);
takeoff_flag = double(takeoff_flag);

% ========================  TUNABLE PARAMETERS  ===========================
Np       = 5;
dt_mpc   = 0.06;

V_NOM    = 0.35;
V_MIN_C  = 0.10;
V_CORNER = 0.4;        % ★ มุมแหลม: ช้ามากจนเลี้ยวทันแน่
V_TURN   = 0.05;
LA_MAX   = 0.14;
LA_MIN   = 0.04;
LA_CORNER = 0.04;      % ★ waypoint แทบจะอยู่ตรงจมูก

B_ALPHA    = 0.60;
CURV_ALPHA = 0.35;
RATE_ALPHA = 0.40;
SNAP_TH    = deg2rad(60);

TH_REV   = pi/2 + 0.2;

% --- Corner detection ---
RATE_CORNER    = deg2rad(60);   % ▼ detect เร็วขึ้น
ERR_CORNER     = 0.35;
CORNER_HOLD    = 15;            % ★ อยู่ใน corner mode อย่างน้อย 15 steps
                                %   (~0.9 วิ ที่ 60 Hz) ป้องกันออก mode เร็ว

Z_TRACK  = -1.1;
ERR_MIN  = 0.20;

% --- MPC cost weights ---
W_CROSS  = 50.0;
W_HEAD   = 2.0;
W_CURV   = 3.0;
W_SPEED  = 0.1;

% --- Directional brake (non-corner only) ---
DIVERGE_TH  = deg2rad(70);
DIVERGE_BRK = 0.3;

% ========================  MAIN LOGIC  ===================================
err_mag = hypot(x_err, y_err);

if takeoff_flag > 0.5 && err_mag > ERR_MIN
    % ------------------------------------------------------------------
    %  1.  RAW BEARING (body frame) — ALWAYS compute this first
    % ------------------------------------------------------------------
    bearing_body = atan2(y_err, x_err);

    % ------------------------------------------------------------------
    %  2.  BEARING RATE → CORNER DETECTION (ก่อน anti-reverse!)
    % ------------------------------------------------------------------
    d_curv_raw = atan2(sin(bearing_body - prev_bearing), ...
                       cos(bearing_body - prev_bearing));
    bear_rate_raw = abs(d_curv_raw);
    bear_rate_smooth = bear_rate_smooth + RATE_ALPHA * (bear_rate_raw - bear_rate_smooth);

    % Corner trigger
    new_corner = (bear_rate_smooth > RATE_CORNER) || ...
                 (bear_rate_smooth > RATE_CORNER * 0.5 && err_mag > ERR_CORNER);

    if new_corner
        corner_cnt = CORNER_HOLD;   % เข้า corner mode + hold
    elseif corner_cnt > 0
        corner_cnt = corner_cnt - 1;
    end

    is_corner = (corner_cnt > 0);

    % ------------------------------------------------------------------
    %  3.  ANTI-REVERSE — ★ DISABLED ระหว่าง CORNER MODE ★
    % ------------------------------------------------------------------
    if ~is_corner
        vmag_body = hypot(vx, vy);
        if vmag_body > V_TURN
            bearing_vel = atan2(vy, vx);
            delta_bv = atan2(sin(bearing_body - bearing_vel), ...
                             cos(bearing_body - bearing_vel));
            if abs(delta_bv) > TH_REV
                bearing_body = bearing_body + pi;
                bearing_body = atan2(sin(bearing_body), cos(bearing_body));
            end
        end
    end
    % ถ้า is_corner → bearing_body = raw จาก vision ไม่ flip

    % ------------------------------------------------------------------
    %  4.  BEARING SMOOTHING
    % ------------------------------------------------------------------
    if is_corner
        % ★ CORNER: SNAP ทันที ไม่ smooth
        bearing_smooth = bearing_body;
    else
        d_bear = atan2(sin(bearing_body - bearing_smooth), ...
                       cos(bearing_body - bearing_smooth));
        if abs(d_bear) > SNAP_TH
            bearing_smooth = bearing_body;
        else
            bearing_smooth = bearing_smooth + B_ALPHA * d_bear;
        end
    end
    bearing_smooth = atan2(sin(bearing_smooth), cos(bearing_smooth));

    % ------------------------------------------------------------------
    %  5.  CURVATURE for adaptive speed (non-corner)
    % ------------------------------------------------------------------
    ds = max(hypot(x_est - prev_x, y_est - prev_y), 1e-4);
    kappa_raw = abs(d_curv_raw) / ds;
    kappa_smooth = kappa_smooth + CURV_ALPHA * (kappa_raw - kappa_smooth);
    kappa_sat = min(max(kappa_smooth, 0.0), 30.0);

    % ------------------------------------------------------------------
    %  6.  SPEED + LOOKAHEAD
    % ------------------------------------------------------------------
    if is_corner
        % ★★★ CORNER MODE: ช้าสุด + waypoint สั้นสุด ★★★
        v_adapt  = V_CORNER;
        la_adapt = LA_CORNER;
    else
        la_adapt = LA_MAX - (LA_MAX - LA_MIN) * (kappa_sat / 30.0);
        v_adapt  = V_NOM  - (V_NOM - V_MIN_C) * (kappa_sat / 30.0);

        % Directional braking (non-corner only)
        vmag_body2 = hypot(vx, vy);
        if vmag_body2 > V_TURN
            bearing_vel2 = atan2(vy, vx);
            div_angle = abs(atan2(sin(bearing_body - bearing_vel2), ...
                                  cos(bearing_body - bearing_vel2)));
            if div_angle > DIVERGE_TH
                v_adapt = v_adapt * DIVERGE_BRK;
            end
        end
    end

    % ------------------------------------------------------------------
    %  7.  WAYPOINT GENERATION
    % ------------------------------------------------------------------
    if is_corner
        % ★ CORNER: ข้าม MPC ทั้งหมด — ชี้ตรงไปหาเส้นที่ vision เห็น
        bearing_ned_out = psi + bearing_body;
        x_planned = x_est + la_adapt * cos(bearing_ned_out);
        y_planned = y_est + la_adapt * sin(bearing_ned_out);
    else
        % NORMAL: two-tier MPC
        % Error target in NED
        err_ned_x = x_err * cos(psi) - y_err * sin(psi);
        err_ned_y = x_err * sin(psi) + y_err * cos(psi);
        tgt_x = x_est + err_ned_x;
        tgt_y = y_est + err_ned_y;

        % Tier 1: coarse 360°
        Nc1 = 9;
        best_cost1 = 1e12;
        best_bear1 = bearing_smooth;
        for ic = 1:Nc1
            frac = (double(ic) - 1.0) / double(Nc1);
            alpha_0 = -pi + 2.0 * pi * frac;
            alpha_0 = atan2(sin(alpha_0), cos(alpha_0));
            c1 = eval_candidate(alpha_0, x_est, y_est, psi, ...
                    v_adapt, dt_mpc, Np, tgt_x, tgt_y, ...
                    bearing_smooth, W_CROSS, W_HEAD, W_CURV, W_SPEED);
            if c1 < best_cost1
                best_cost1 = c1;
                best_bear1 = alpha_0;
            end
        end

        % Tier 2: fine ±25° around winner
        Nc2 = 13;
        fine_range = deg2rad(25);
        best_cost2 = 1e12;
        best_bear2 = best_bear1;
        for ic = 1:Nc2
            frac = (double(ic) - 1.0) / (double(Nc2) - 1.0);
            offset = -fine_range + 2.0 * fine_range * frac;
            alpha_0 = best_bear1 + offset;
            alpha_0 = atan2(sin(alpha_0), cos(alpha_0));
            c2 = eval_candidate(alpha_0, x_est, y_est, psi, ...
                    v_adapt, dt_mpc, Np, tgt_x, tgt_y, ...
                    bearing_smooth, W_CROSS, W_HEAD, W_CURV, W_SPEED);
            if c2 < best_cost2
                best_cost2 = c2;
                best_bear2 = alpha_0;
            end
        end

        bearing_ned_out = psi + best_bear2;
        x_planned = x_est + la_adapt * cos(bearing_ned_out);
        y_planned = y_est + la_adapt * sin(bearing_ned_out);
    end

    yaw_planned = psi;

    % ------------------------------------------------------------------
    %  8.  UPDATE STATES
    % ------------------------------------------------------------------
    prev_bearing = bearing_smooth;
    prev_x       = x_est;
    prev_y       = y_est;

else
    x_planned        = x_est;
    y_planned        = y_est;
    yaw_planned      = psi;
    bearing_smooth   = 0.0;
    prev_bearing     = 0.0;
    prev_x           = x_est;
    prev_y           = y_est;
    bear_rate_smooth = 0.0;
    corner_cnt       = 0;
end

z_planned = Z_TRACK;
end

% =====================================================================
%  HELPER: evaluate one MPC candidate (geometric cost)
% =====================================================================
function cost = eval_candidate(alpha_0, x0, y0, psi, ...
    v_sim, dt, Np, tgt_x, tgt_y, ...
    bearing_ref, W_CR, W_HD, W_CV, W_SP)
%#codegen
px = x0;
py = y0;
cost = 0.0;
for kk = 1:Np
    heading_ned = psi + alpha_0;
    px = px + v_sim * cos(heading_ned) * dt;
    py = py + v_sim * sin(heading_ned) * dt;
    dx = tgt_x - px;
    dy = tgt_y - py;
    dist = hypot(dx, dy);
    cross_cost = dist * dist;
    head_err = atan2(sin(alpha_0 - bearing_ref), ...
                     cos(alpha_0 - bearing_ref));
    head_cost = abs(head_err);
    if kk == 1
        cv = abs(atan2(sin(alpha_0 - bearing_ref), ...
                        cos(alpha_0 - bearing_ref)));
    else
        cv = 0.0;
    end
    prog = v_sim * dt * cos(atan2(dy, dx) - heading_ned);
    cost = cost + W_CR*cross_cost + W_HD*head_cost + W_CV*cv - W_SP*prog;
end
end
