function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
%#codegen

persistent cdir cinit circle_cnt track_frames
if isempty(cdir);  cdir  = 0.0; end
if isempty(cinit); cinit = 0.0; end
if isempty(circle_cnt); circle_cnt = 0; end
if isempty(track_frames); track_frames = 0; end

STEP_LEN = 2.0;
N_STEP   = 12;
WIN_R    = 15;
TURN_MAX = 2.4;
DIR_LP   = 0.20;
MAX_SLEW = 0.80;

CIRC_R    = 15;
CIRC_TH   = 500;
CIRC_HOLD = 3;

orientation_track = atan2(-x_ref_prev, y_ref_prev);
orientation_track = mod(orientation_track, 2*pi);
imax = size(frame,1); jmax = size(frame,2);
error_x_track = 0; error_y_track = 0; flag_track = false;

if track_frames > 15
    circ_total = 0.0;
    for ic = max(1,COG_X-CIRC_R):min(imax,COG_X+CIRC_R)
        for jc = max(1,COG_Y-CIRC_R):min(jmax,COG_Y+CIRC_R)
            if frame(ic,jc)==1
                if (ic-COG_X)^2+(jc-COG_Y)^2 <= CIRC_R*CIRC_R
                    circ_total = circ_total + 1.0;
                end
            end
        end
    end
    if circ_total > CIRC_TH; circle_cnt = circle_cnt + 1;
    else;                     circle_cnt = 0;
    end
    if circle_cnt >= CIRC_HOLD; return; end
end

if flag_track_prev == 0
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
    if cinit == 0.0
        cdir  = orientation_track;
        cinit = 1.0;
    end

    ci = COG_X - 0.5; cj = COG_Y - 0.5;
    mdir = cdir; nf = 0.0;
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
        if cnt < 1.0; break; end
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
        track_frames  = track_frames + 1;
        net = atan2(ci-(COG_X-0.5), cj-(COG_Y-0.5));
        dn  = atan2(sin(net-cdir), cos(net-cdir));
        if dn >  MAX_SLEW; dn =  MAX_SLEW; end
        if dn < -MAX_SLEW; dn = -MAX_SLEW; end
        cdir = cdir + dn;
    end
end

end
