function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
% LINE-TRACING detector (connected + predictive) + COMMITTED-DIRECTION guard.
%
% Marches along the connected line from the drone, heading rotates to follow the
% line's curve.  A PERSISTENT committed travel direction (cdir) seeds the march
% and is updated each frame but SLEW-LIMITED (max MAX_SLEW per frame) so the
% travel direction stays CONTINUOUS:
%   - a gradual bend (incl. backward) is followed  (cdir rotates over frames)
%   - a sudden 180 deg reversal onto the came-from arm is BLOCKED (cdir can't
%     flip in one frame) -> the drone no longer runs back the way it came.
% I/O identical to the block.  paste over the whole "waypoints 1" block body.

%#codegen

persistent cdir cinit
if isempty(cdir);  cdir  = 0.0; end
if isempty(cinit); cinit = 0.0; end

% ── parameters ────────────────────────────────────────────────────────────
STEP_LEN = 2.0;    % march step length [px]
N_STEP   = 12;     % steps (look-ahead ~= STEP_LEN*N_STEP px)
WIN_R    = 15;      % search half-window around each step [px]   (15 กว้างไป->โดนแขนหลัง)
TURN_MAX = 2.4;    % rad ~137deg : ตามโค้งได้ แต่ตัดทางหลัง(180°)ทิ้ง
DIR_LP   = 0.20;   % heading low-pass per step                   (พี่จูน)
MAX_SLEW = 0.80;   % rad/frame : max change of committed dir (กันย้อน - ลดถ้ายังย้อน)

orientation_track = atan2(-x_ref_prev, y_ref_prev);
orientation_track = mod(orientation_track, 2*pi);

imax = size(frame,1);
jmax = size(frame,2);
error_x_track = 0;
error_y_track = 0;
flag_track    = false;

if flag_track_prev == 0
    % ── INITIAL acquisition: crown-ring centroid + reseed committed dir ───
    cinit = 0.0;
    sumi = 0.0; sumj = 0.0; cnt = 0.0;
    for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)
        for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)
            if i>=1 && i<=imax && j>=1 && j<=jmax
                nrm = sqrt((i-(COG_X-0.5))^2 + (j-(COG_Y-0.5))^2);
                if nrm>=MIN_RADIUS_CROWN && nrm<=MAX_RADIUS_CROWN && frame(i,j)==1
                    sumi = sumi+i; sumj = sumj+j; cnt = cnt+1.0;
                end
            end
        end
    end
    if cnt > 0.0
        error_x_track = -(round(sumi/cnt) - COG_X);
        error_y_track =  (round(sumj/cnt) - COG_Y);
        flag_track    = true;
    end

else
    % ── seed committed direction (first locked frame) ─────────────────────
    if cinit == 0.0
        cdir  = orientation_track;
        cinit = 1.0;
    end

    % ── MARCH along the connected line, starting from the committed dir ───
    ci = COG_X - 0.5;
    cj = COG_Y - 0.5;
    mdir = cdir;
    nf = 0.0;
    for s = 1:N_STEP
        ti = ci + STEP_LEN*sin(mdir);
        tj = cj + STEP_LEN*cos(mdir);
        sumi = 0.0; sumj = 0.0; cnt = 0.0;
        for ii = floor(ti-WIN_R):ceil(ti+WIN_R)
            for jj = floor(tj-WIN_R):ceil(tj+WIN_R)
                if ii>=1 && ii<=imax && jj>=1 && jj<=jmax && frame(ii,jj)==1
                    ang = atan2(ii-ci, jj-cj);
                    dev = abs(atan2(sin(ang-mdir), cos(ang-mdir)));
                    if dev <= TURN_MAX
                        sumi = sumi+ii; sumj = sumj+jj; cnt = cnt+1.0;
                    end
                end
            end
        end
        if cnt < 1.0
            break;
        end
        ni = sumi/cnt; nj = sumj/cnt;
        nd = atan2(ni-ci, nj-cj);
        mdir = mdir + DIR_LP*atan2(sin(nd-mdir), cos(nd-mdir));
        ci = ni; cj = nj;
        nf = nf + 1.0;
    end

    if nf > 0.0
        error_x_track = -(round(ci) - COG_X);
        error_y_track =  (round(cj) - COG_Y);
        flag_track    = true;
        % update committed direction toward the look-ahead, SLEW-LIMITED
        net = atan2(ci-(COG_X-0.5), cj-(COG_Y-0.5));
        dn  = atan2(sin(net-cdir), cos(net-cdir));
        if dn >  MAX_SLEW; dn =  MAX_SLEW; end
        if dn < -MAX_SLEW; dn = -MAX_SLEW; end
        cdir = cdir + dn;
    end
end

end
