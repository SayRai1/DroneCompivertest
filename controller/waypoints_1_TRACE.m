function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
% LINE-TRACING detector (connected + predictive).
%
% Instead of "pick an arm by angle", it MARCHES along the connected line:
% starting at the drone centre, heading = the previous lane direction, it steps
% forward and at each step snaps to the local line and rotates its heading to
% follow the line's curve.  Because it walks ALONG one connected line:
%   - it never jumps to a different (disconnected) arm  -> no fork flipping
%   - it follows the line wherever it bends, incl. backward (the heading rotates
%     with the line; only a near-full reversal > TURN_MAX is blocked)
% The "lock" persists through the model feedback (orientation_track comes from
% the previous output via the z^-1 blocks).  I/O identical to the block.
%
% paste over the whole "waypoints 1" block body.

%#codegen

% ── parameters ────────────────────────────────────────────────────────────
STEP_LEN = 2.0;    % march step length [px]
N_STEP   = 12;     % number of steps  (look-ahead ~= STEP_LEN*N_STEP px)
WIN_R    = 4;      % search half-window around each step [px]
TURN_MAX = 2.36;   % rad ~135deg : max turn per step (blocks only the ~back)
DIR_LP   = 0.5;    % heading low-pass per step (1 = snap hard to local line)

% lane heading from the previous tracking point (= march seed, and an output)
orientation_track = atan2(-x_ref_prev, y_ref_prev);
orientation_track = mod(orientation_track, 2*pi);

imax = size(frame,1);
jmax = size(frame,2);
error_x_track = 0;
error_y_track = 0;
flag_track    = false;

if flag_track_prev == 0
    % ── INITIAL acquisition: plain crown-ring centroid (detect line @ takeoff)
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
    % ── MARCH along the connected line ────────────────────────────────────
    ci   = COG_X - 0.5;          % current march position (row, col)
    cj   = COG_Y - 0.5;
    mdir = orientation_track;    % start heading = previous lane direction
    nf   = 0.0;

    for s = 1:N_STEP
        % candidate point one step ahead along the current heading
        ti = ci + STEP_LEN*sin(mdir);
        tj = cj + STEP_LEN*cos(mdir);
        % centroid of line pixels in the window that lie in the forward cone
        sumi = 0.0; sumj = 0.0; cnt = 0.0;
        for ii = floor(ti-WIN_R):ceil(ti+WIN_R)
            for jj = floor(tj-WIN_R):ceil(tj+WIN_R)
                if ii>=1 && ii<=imax && jj>=1 && jj<=jmax && frame(ii,jj)==1
                    ang = atan2(ii-ci, jj-cj);                 % current pos -> pixel
                    dev = abs(atan2(sin(ang-mdir), cos(ang-mdir)));
                    if dev <= TURN_MAX
                        sumi = sumi+ii; sumj = sumj+jj; cnt = cnt+1.0;
                    end
                end
            end
        end
        if cnt < 1.0
            break;                          % line ended ahead -> stop
        end
        ni = sumi/cnt; nj = sumj/cnt;        % local line centre
        nd = atan2(ni-ci, nj-cj);            % heading toward it
        mdir = mdir + DIR_LP*atan2(sin(nd-mdir), cos(nd-mdir));   % rotate to follow
        ci = ni; cj = nj;                    % advance along the line
        nf = nf + 1.0;
    end

    if nf > 0.0
        error_x_track = -(round(ci) - COG_X);
        error_y_track =  (round(cj) - COG_Y);
        flag_track    = true;
    end
end

end
