function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
% VARIANT A: VELOCITY-BASED anti-reverse guard.
%
% รูปแบบ crab เหมือนเดิม (ไม่มี yaw) แต่ใช้ "ทิศที่โดรนกำลังบินจริง" จาก vx,vy
% เป็น reference ของ "หน้า" ถ้า bearing จาก detector ทำมุมกับทิศบินจริง > 90°
% = โดรนถูกลากย้อน -> FLIP bearing 180° กลับให้ตรงทิศวิ่งจริง
%
% ใช้ ground-truth ของการเคลื่อนที่จริง ไม่ใช่ภาพ -> ไม่หลอกด้วย back-bend
%#codegen
persistent bearing_smooth was_tracking land_cnt z_cmd
if isempty(bearing_smooth)
    bearing_smooth = 0.0;
    was_tracking = 0;
    land_cnt = 0;
    z_cmd = -1.1;
end
x_err=double(x_err); y_err=double(y_err); psi=double(psi);
vx=double(vx); vy=double(vy);
x_est=double(x_est); y_est=double(y_est); takeoff_flag=double(takeoff_flag);

LOOKAHEAD = 0.12;
L_SLOW    = 0.12;
TH_SLOW   = deg2rad(18);
B_ALPHA   = 0.60;
ERR_MIN   = 0.2;
Z_TRACK   = -1.1;
V_MIN     = 0.07;   % m/s : ใช้ velocity ก็ต่อเมื่อบินจริงๆ (ไม่ใช่ hover noise)
TH_REV    = pi/2 +0.2;   % bearing ห่างจากทิศบินจริงเกินนี้ = ย้อน -> flip
LAND_WAIT = 200;         % 200 frames = 1s @200Hz ก่อนเริ่มลง
DESCENT   = 0.005;     % m/frame ≈ 0.15 m/s descent

err_mag = hypot(x_err, y_err);
if takeoff_flag && err_mag > ERR_MIN
    bearing_body = atan2(y_err, x_err);

    % --- VELOCITY-BASED ANTI-REVERSE -------------------------------------
    % vx,vy body -> NED ; แปลง NED -> body bearing (ที่โดรน "รู้สึก" ว่าหน้า)
    vN = vx*cos(psi) - vy*sin(psi);
    vE = vx*sin(psi) + vy*cos(psi);
    vmag = hypot(vN, vE);
    if vmag > V_MIN
        % ทิศบินจริงใน body frame = หักล้าง psi ออกแล้ว = body forward
        bearing_vel = atan2(vy, vx);            % body-frame velocity heading
        % ถ้า bearing จาก detector ทวนทิศบินจริง > 90° -> flip 180°
        dv = abs(atan2(sin(bearing_body-bearing_vel), cos(bearing_body-bearing_vel)));
        if dv > TH_REV
            bearing_body = bearing_body + pi;
            bearing_body = atan2(sin(bearing_body), cos(bearing_body));
        end
    end

    d = atan2(sin(bearing_body - bearing_smooth), cos(bearing_body - bearing_smooth));
    if abs(d) > (pi/2)
        bearing_smooth = bearing_body;
    else
        bearing_smooth = bearing_smooth + B_ALPHA * d;
    end
    v = LOOKAHEAD;
    if abs(bearing_smooth) > TH_SLOW; v = L_SLOW; end
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
        z_cmd = 0.0;
    end
end

z_planned = z_cmd;
end
