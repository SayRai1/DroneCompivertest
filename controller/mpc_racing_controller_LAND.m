function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
%#codegen
persistent bearing_smooth was_tracking land_cnt z_cmd fly_timer
if isempty(bearing_smooth)
    bearing_smooth = 0.0;
    was_tracking = 0;
    land_cnt = 0;
    z_cmd = -1.1;
    fly_timer = 0.0;
end

x_err=double(x_err); y_err=double(y_err); psi=double(psi);
vx=double(vx); vy=double(vy);
x_est=double(x_est); y_est=double(y_est); takeoff_flag=double(takeoff_flag);

LOOKAHEAD = 0.12;
L_SLOW    = 0.11;
TH_SLOW   = deg2rad(18);
B_ALPHA   = 0.60;
ERR_MIN   = 0.2;
Z_TRACK   = -1.1;
V_MIN     = 0.11;
TH_REV    = pi/2 + 0.2;
LAND_WAIT = 280;
DESCENT   = 0.00075;
DT        = 0.005;
T_MIN_FLY = 15.0;

if takeoff_flag > 0.5; fly_timer = fly_timer + DT; end

err_mag = hypot(x_err, y_err);
if takeoff_flag && err_mag > ERR_MIN
    bearing_body = atan2(y_err, x_err);

    vN = vx*cos(psi) - vy*sin(psi);
    vE = vx*sin(psi) + vy*cos(psi);
    vmag = hypot(vN, vE);
    if vmag > V_MIN
        bearing_vel = atan2(vy, vx);
        dv = abs(atan2(sin(bearing_body-bearing_vel), cos(bearing_body-bearing_vel)));
        if dv > TH_REV
            bearing_body = bearing_body + pi;
            bearing_body = atan2(sin(bearing_body), cos(bearing_body));
        end
    end

    d = atan2(sin(bearing_body - bearing_smooth), cos(bearing_body - bearing_smooth));
    if abs(d) > (pi/2)
        bearing_smooth = bearing_body;
        v = 0.03;
    else
        bearing_smooth = bearing_smooth + B_ALPHA * d;
        v = LOOKAHEAD;
        if abs(bearing_smooth) > TH_SLOW; v = L_SLOW; end
    end
    bearing_ned = psi + bearing_smooth;
    x_planned   = x_est + v*cos(bearing_ned);
    y_planned   = y_est + v*sin(bearing_ned);
    yaw_planned = psi;
    was_tracking = 1;
    land_cnt = 0;
    z_cmd = Z_TRACK;
else
    x_planned=x_est; y_planned=y_est; yaw_planned=psi;
    bearing_smooth=0.0;
    if takeoff_flag > 0.5 && was_tracking > 0.5
        land_cnt = land_cnt + 1;
    end
end
if land_cnt > LAND_WAIT && fly_timer > T_MIN_FLY
    z_cmd = min(z_cmd + DESCENT, 0.0);
end
z_planned = z_cmd;
end
