function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
%#codegen

persistent cdir cinit circle_lock track_frames
if isempty(cdir);  cdir  = 0.0; end
if isempty(cinit); cinit = 0.0; end
if isempty(circle_lock); circle_lock = 0; end
if isempty(track_frames); track_frames = 0; end

STEP_LEN = 2.0;
N_STEP   = 24;
WIN_R    = 14;
TURN_MAX = 2.5;
DIR_LP   = 0.20;
MAX_SLEW = 0.70;

orientation_track = atan2(-x_ref_prev, y_ref_prev);
orientation_track = mod(orientation_track, 2*pi);

imax = size(frame,1);
jmax = size(frame,2);
error_x_track = 0;
error_y_track = 0;
flag_track    = false;

% ── CIRCLE LOCK: ถ้าล็อคแล้ว → return centroid เสมอ ──────────────────────
if circle_lock > 0.5
    r1 = max(1,COG_X-20); r2 = min(imax,COG_X+20);
    c1 = max(1,COG_Y-20); c2 = min(jmax,COG_Y+20);
    si = 0.0; sj = 0.0; sc = 0.0;
    for i = r1:r2
        for j = c1:c2
            if frame(i,j) == 1
                si = si + i; sj = sj + j; sc = sc + 1.0;
            end
        end
    end
    if sc > 0
        error_x_track = -(round(si/sc) - COG_X);
        error_y_track =  (round(sj/sc) - COG_Y);
    end
    flag_track = true;
    return;
end

% ── COLD START ────────────────────────────────────────────────────────────
if flag_track_prev == 0
    cinit = 0.0;
    track_frames = 0;
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

% ── TRACKING (MARCH) ─────────────────────────────────────────────────────
else
    track_frames = track_frames + 1;

    % +++ blob safety: ดักไม่ให้ march ทะลุกลุ่ม pixel หนาแน่น +++
    r1 = max(1,COG_X-20); r2 = min(imax,COG_X+20);
    c1 = max(1,COG_Y-20); c2 = min(jmax,COG_Y+20);
    blob_sum = sum(sum(frame(r1:r2, c1:c2)));

    % ── circle detection: quadrant check (หลัง track >= 15 frame) ─────────
    CIRC_DET_R = 15;
    CIRC_Q_MIN = 40;
    MIN_TRACK  = 15;
    if blob_sum > 750 && track_frames > MIN_TRACK
        q1=0.0; q2=0.0; q3=0.0; q4=0.0;
        for ic = max(1,COG_X-CIRC_DET_R):min(imax,COG_X+CIRC_DET_R)
            for jc = max(1,COG_Y-CIRC_DET_R):min(jmax,COG_Y+CIRC_DET_R)
                if frame(ic,jc)==1 && (ic-COG_X)^2+(jc-COG_Y)^2 <= CIRC_DET_R*CIRC_DET_R
                    if ic <= COG_X && jc <= COG_Y; q1 = q1+1; end
                    if ic <= COG_X && jc >  COG_Y; q2 = q2+1; end
                    if ic >  COG_X && jc <= COG_Y; q3 = q3+1; end
                    if ic >  COG_X && jc >  COG_Y; q4 = q4+1; end
                end
            end
        end
        if q1>CIRC_Q_MIN && q2>CIRC_Q_MIN && q3>CIRC_Q_MIN && q4>CIRC_Q_MIN
            circle_lock = 1;
        end
    end

    % blob freeze: hover แทน march เมื่อเจอกลุ่มหนาแน่น
    if blob_sum > 750
        si = 0.0; sj = 0.0; sc = 0.0;
        for i = r1:r2
            for j = c1:c2
                if frame(i,j) == 1
                    si = si + i; sj = sj + j; sc = sc + 1.0;
                end
            end
        end
        if sc > 0
            error_x_track = -(round(si/sc) - COG_X);
            error_y_track =  (round(sj/sc) - COG_Y);
        end
        flag_track = true;
        return;
    end

    % ── seed committed direction (first locked frame) ─────────────────────
    if cinit == 0.0
        cdir  = orientation_track;
        cinit = 1.0;
    end

    % ── MARCH along the connected line ───────────────────────────────────
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
        net = atan2(ci-(COG_X-0.5), cj-(COG_Y-0.5));
        dn  = atan2(sin(net-cdir), cos(net-cdir));
        if dn >  MAX_SLEW; dn =  MAX_SLEW; end
        if dn < -MAX_SLEW; dn = -MAX_SLEW; end
        cdir = cdir + dn;
    end
end

end
