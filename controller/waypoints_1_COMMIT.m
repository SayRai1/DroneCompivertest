function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
% COMMITTED-DIRECTION track detector (fixes "can't decide which way at a fork").
%
% vs plain branch-pick it adds two things:
%   (1) a PERSISTENT committed travel direction dir_lock (low-pass) - the chosen
%       branch can no longer flip frame-to-frame, so the drone stops dithering.
%   (2) BEHIND-EXCLUSION - track pixels in a wedge around the came-from
%       direction (dir_lock + pi) are dropped, so at a corner / hairpin the arm
%       the drone arrived on (and a nearby anti-parallel return arm) can never be
%       chosen; only the FORWARD continuation is.
% I/O identical to the original block -> paste over the whole block body.

persistent x_accumulator
persistent y_accumulator
persistent pixel_count
persistent dir_lock          % committed travel direction [rad] (image frame)
persistent dir_init          % 1 once dir_lock has been seeded this run

if isempty(x_accumulator); x_accumulator = 0; end
if isempty(y_accumulator); y_accumulator = 0; end
if isempty(pixel_count);   pixel_count   = 0; end
if isempty(dir_lock);      dir_lock      = 0; end
if isempty(dir_init);      dir_init      = 0; end

% heading of the lane from the previous tracking point (kept as an output)
orientation_track = atan2(-x_ref_prev, y_ref_prev);
orientation_track = mod(orientation_track, 2*pi);

CLUSTER_HALF = pi/4;     % one branch spans +-45 deg
BEHIND_HALF  = pi/3;     % drop +-60 deg around the came-from direction
DIR_ALPHA    = 0.25;     % committed-direction low-pass (small = stickier)

if flag_track_prev

    % seed the committed direction once, from the previous lane heading
    if dir_init == 0
        dir_lock = orientation_track;
        dir_init = 1;
    end
    ref_dir = dir_lock;

    % ---- PASS 1: forward branch nearest ref_dir, excluding the behind wedge --
    best_dev   = 100;
    branch_yaw = ref_dir;
    found      = false;
    for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)
        for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)
            nrm = sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));
            if nrm >= MIN_RADIUS_CROWN && nrm <= MAX_RADIUS_CROWN
                if frame(i,j) == 1
                    axis_x = (i-(COG_X-0.5));
                    axis_y = (j-(COG_Y-0.5));
                    yaw_pt = atan2(axis_x, axis_y);
                    dev_back = abs(atan2(sin(yaw_pt-ref_dir-pi), cos(yaw_pt-ref_dir-pi)));
                    if dev_back > BEHIND_HALF          % not the came-from wedge
                        dev = abs(atan2(sin(yaw_pt-ref_dir), cos(yaw_pt-ref_dir)));
                        if dev < best_dev
                            best_dev   = dev;
                            branch_yaw = yaw_pt;
                            found      = true;
                        end
                    end
                end
            end
        end
    end

    % ---- PASS 2: accumulate only that branch's pixels ----------------------
    if found
        for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)
            for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)
                nrm = sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));
                if nrm >= MIN_RADIUS_CROWN && nrm <= MAX_RADIUS_CROWN
                    if frame(i,j) == 1
                        axis_x = (i-(COG_X-0.5));
                        axis_y = (j-(COG_Y-0.5));
                        yaw_pt = atan2(axis_x, axis_y);
                        dev_b = abs(atan2(sin(yaw_pt-branch_yaw), cos(yaw_pt-branch_yaw)));
                        if dev_b <= CLUSTER_HALF
                            pixel_count   = pixel_count + 1;
                            x_accumulator = x_accumulator + i;
                            y_accumulator = y_accumulator + j;
                        end
                    end
                end
            end
        end
        % commit: low-pass dir_lock toward the chosen branch
        dlk      = atan2(sin(branch_yaw-dir_lock), cos(branch_yaw-dir_lock));
        dir_lock = dir_lock + DIR_ALPHA*dlk;
    end

else
    % initial heading search (not yet locked): full ring, re-seed next lock
    for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)
        for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)
            nrm = sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));
            if nrm >= MIN_RADIUS_CROWN && nrm <= MAX_RADIUS_CROWN
                if frame(i,j) == 1
                    pixel_count   = pixel_count + 1;
                    x_accumulator = x_accumulator + i;
                    y_accumulator = y_accumulator + j;
                end
            end
        end
    end
    dir_init = 0;
end

% centroid of the selected branch
x_ref_temp = -(round(x_accumulator/pixel_count) - COG_X);
y_ref_temp =  (round(y_accumulator/pixel_count) - COG_Y);

if isnan(x_ref_temp)
    error_x_track = 0;     flag_error_x = false;
else
    flag_error_x  = true;  error_x_track = x_ref_temp;
end
if isnan(y_ref_temp)
    error_y_track = 0;     flag_error_y = false;
else
    error_y_track = y_ref_temp; flag_error_y = true;
end
flag_track = flag_error_x || flag_error_y;

pixel_count = 0; x_accumulator = 0; y_accumulator = 0;
end
