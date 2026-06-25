function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
% Holonomic crab + corner detector + Landing Method C (Timer & Track-loss).
%#codegen

persistent bearing_smooth prev_bearing corner_cnt ...
           fly_timer lost_cnt do_land first_track z_cmd
if isempty(bearing_smooth)
    bearing_smooth = 0.0;
    prev_bearing   = 0.0;
    corner_cnt     = 0;
    fly_timer      = 0.0;
    lost_cnt       = 0;
    do_land        = 0;
    first_track    = 1;
    z_cmd          = -1.1;
end

x_err=double(x_err); y_err=double(y_err); psi=double(psi);
vx=double(vx); vy=double(vy);
x_est=double(x_est); y_est=double(y_est); takeoff_flag=double(takeoff_flag);

LOOKAHEAD = 0.14;
L_SLOW    = 0.11;
L_CORNER  = 0.13;
TH_SLOW   = deg2rad(18);
MAX_RATE   = 0.04;
ERR_MIN    = 0.3;
Z_TRACK    = -1.1;
DT         = 0.005;
CORNER_TH  = deg2rad(35);
CORNER_HOLD= 200;

T_LAND     = 30.0;
LOST_MAX   = 200;
T_STARTUP  = 5.0;
DESCENT    = 0.00075;

if takeoff_flag > 0.5
    fly_timer = fly_timer + DT;
end

err_mag   = hypot(x_err, y_err);
has_track = (takeoff_flag > 0.5) && (err_mag > ERR_MIN);

if has_track
    lost_cnt = 0;
elseif takeoff_flag > 0.5
    lost_cnt = lost_cnt + 1;
end

if do_land < 0.5
    if fly_timer > T_LAND
        do_land = 1;
    end
    if lost_cnt > LOST_MAX && fly_timer > T_STARTUP
        do_land = 1;
    end
end

if do_land > 0.5
    z_cmd = min(z_cmd + DESCENT, 0.0);
    z_planned = z_cmd;
else
    z_cmd = Z_TRACK;
    z_planned = Z_TRACK;
end

if has_track && do_land < 0.5
    bearing_body = atan2(y_err, x_err);

    if first_track > 0.5
        prev_bearing = bearing_body;
        first_track  = 0;
    end

    d_jump = abs(atan2(sin(bearing_body - prev_bearing), ...
                       cos(bearing_body - prev_bearing)));
    if d_jump > CORNER_TH
        corner_cnt = CORNER_HOLD;
    elseif corner_cnt > 0
        corner_cnt = corner_cnt - 1;
    end
    is_corner = (corner_cnt > 0);
    prev_bearing = bearing_body;

    if is_corner
        bearing_smooth = bearing_body;
    else
        d = atan2(sin(bearing_body - bearing_smooth), ...
                  cos(bearing_body - bearing_smooth));
        if d > MAX_RATE;  d =  MAX_RATE; end
        if d < -MAX_RATE; d = -MAX_RATE; end
        bearing_smooth = bearing_smooth + d;
    end
    bearing_smooth = atan2(sin(bearing_smooth), cos(bearing_smooth));

    if is_corner
        v = L_CORNER;
    elseif abs(bearing_smooth) > TH_SLOW
        v = L_SLOW;
    else
        v = LOOKAHEAD;
    end

    bearing_ned = psi + bearing_smooth;
    x_planned   = x_est + v * cos(bearing_ned);
    y_planned   = y_est + v * sin(bearing_ned);
    yaw_planned = psi;
else
    x_planned   = x_est;
    y_planned   = y_est;
    yaw_planned = psi;
    if takeoff_flag < 0.5
        bearing_smooth = 0.0;
        prev_bearing   = 0.0;
        corner_cnt     = 0;
        fly_timer      = 0.0;
        lost_cnt       = 0;
        do_land        = 0;
        first_track    = 1;
        z_cmd          = -1.1;
    end
end
end
