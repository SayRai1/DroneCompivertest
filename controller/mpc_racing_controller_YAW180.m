function [x_planned, y_planned, z_planned, yaw_planned] = mpc_racing_controller( ...
        x_err, y_err, phi, theta, psi, ...
        vx, vy, vz, p, q, r, takeoff_flag, ...
        x_est, y_est)
% VARIANT B: YAW-FLIP เมื่อเจอ back-bend (หักเกิน 180°).
%
% State machine:
%   STRAIGHT : crab + hold heading (เหมือนเดิม)
%   FLIPPING : bearing > TH_FLIP (~150°) ค้าง DWELL เฟรม -> สั่งหมุน yaw 180°
%              hover (v=0) จนหมุนถึง yaw_goal ภายใน TH_DONE แล้วกลับ STRAIGHT
%   หลัง flip: กล้องเห็นทาง "ที่เคยอยู่หลัง" เป็น "หน้า" -> bearing เล็ก -> crab ปกติ
%
% หมุนเฉพาะตอน confirmed back-bend (dwell counter) -> ปกติทางตรง/โค้งเลี้ยวไม่หมุน
%#codegen
persistent bearing_smooth flip_count yaw_goal mode
if isempty(bearing_smooth); bearing_smooth=0.0; end
if isempty(flip_count);     flip_count=0.0;     end
if isempty(yaw_goal);       yaw_goal=0.0;       end
if isempty(mode);           mode=0.0;           end   % 0=STRAIGHT 1=FLIPPING

x_err=double(x_err); y_err=double(y_err); psi=double(psi);
x_est=double(x_est); y_est=double(y_est); takeoff_flag=double(takeoff_flag);

LOOKAHEAD = 0.12;
L_SLOW    = 0.11;
TH_SLOW   = deg2rad(18);
B_ALPHA   = 0.20;
ERR_MIN   = 0.3;
Z_TRACK   = -1.1;
TH_FLIP   = deg2rad(150);   % bearing เกินนี้ = สงสัยว่าเป็น back-bend
DWELL     = 3;              % เฟรมที่ต้องเกินค้าง ก่อน confirm flip
TH_DONE   = deg2rad(15);    % yaw ห่าง goal น้อยกว่านี้ = หมุนเสร็จ

err_mag = hypot(x_err, y_err);
if takeoff_flag && err_mag > ERR_MIN
    bearing_body = atan2(y_err, x_err);

    if mode < 0.5
        % --- STRAIGHT: ตรวจจับ back-bend ---------------------------------
        if abs(bearing_body) > TH_FLIP
            flip_count = flip_count + 1.0;
        else
            flip_count = 0.0;
        end
        if flip_count >= DWELL
            mode     = 1.0;
            yaw_goal = atan2(sin(psi+pi), cos(psi+pi));   % หมุน 180°
            flip_count = 0.0;
        end
    else
        % --- FLIPPING: ตรวจว่าหมุนเสร็จยัง --------------------------------
        de = abs(atan2(sin(yaw_goal-psi), cos(yaw_goal-psi)));
        if de < TH_DONE
            mode = 0.0;
            bearing_smooth = 0.0;
        end
    end

    if mode > 0.5
        % --- FLIPPING: hover + สั่ง yaw 180° -----------------------------
        x_planned   = x_est;
        y_planned   = y_est;
        yaw_planned = atan2(sin(yaw_goal), cos(yaw_goal));
    else
        % --- STRAIGHT: crab ปกติ -----------------------------------------
        d = atan2(sin(bearing_body-bearing_smooth), cos(bearing_body-bearing_smooth));
        if abs(d) > (pi/2); bearing_smooth = bearing_body;
        else; bearing_smooth = bearing_smooth + B_ALPHA*d; end
        v = LOOKAHEAD;
        if abs(bearing_smooth) > TH_SLOW; v = L_SLOW; end
        bearing_ned = psi + bearing_smooth;
        x_planned   = x_est + v*cos(bearing_ned);
        y_planned   = y_est + v*sin(bearing_ned);
        yaw_planned = psi;
    end
else
    x_planned=x_est; y_planned=y_est; yaw_planned=psi;
    bearing_smooth=0.0; flip_count=0.0; mode=0.0; yaw_goal=psi;
end
z_planned = Z_TRACK;
end
