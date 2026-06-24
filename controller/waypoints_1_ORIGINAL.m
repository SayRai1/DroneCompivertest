function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
% BRANCH-PICKING version of the crown-mask track detector.
%
% WHY: the original averages EVERY track pixel inside the crown+FOV. At a
% 90 deg corner that averages the "straight" arm and the "turn" arm, so the
% tracking point lands on the inside of the corner - straight ahead of the
% drone. The controller then has no lateral target, so the drone just hovers
% and refuses to turn ("phyayam song tua, mai yom liao").
%
% FIX (change the tracking point): pick a SINGLE branch. Pass 1 finds the
% track bearing closest to the current heading; Pass 2 keeps only the pixels
% of that one arm and uses their centroid. While the straight arm exists the
% drone commits to it and drives to the vertex; once it reaches the vertex the
% straight arm is gone, the only branch left is the turn, so the tracking
% point jumps onto the turn and the drone follows it round.
%
% I/O is identical to the original block - paste over the whole block body,
% no rewiring needed.

persistent x_accumulator
persistent y_accumulator
persistent pixel_count

if isempty(x_accumulator)
    x_accumulator = 0;
end
if isempty(y_accumulator)
    y_accumulator = 0;
end
if isempty(pixel_count)
    pixel_count = 0;
end

% previous heading angle of the lane (forward direction along the track)
orientation_track = atan2(-x_ref_prev, y_ref_prev);
orientation_track = mod(orientation_track, 2*pi);

ARC_HALF     = pi/FOV;   % half field-of-view [rad] (same meaning as original)
CLUSTER_HALF = pi/4;     % one branch spans +-45 deg; separates perpendicular arms

if flag_track_prev

    % ---- PASS 1: bearing of the branch closest to the current heading ----
    best_dev   = 100;                % larger than any possible deviation (<= pi)
    branch_yaw = orientation_track;  % default if nothing is found
    found      = false;

    for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)
        for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)
            nrm = sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));
            if nrm >= MIN_RADIUS_CROWN && nrm <= MAX_RADIUS_CROWN
                if frame(i,j) == 1
                    axis_x = (i-(COG_X-0.5));
                    axis_y = (j-(COG_Y-0.5));
                    yaw_pt = atan2(axis_x, axis_y);
                    dev = abs(atan2(sin(yaw_pt-orientation_track), ...
                                    cos(yaw_pt-orientation_track)));
                    if dev <= ARC_HALF && dev < best_dev
                        best_dev   = dev;
                        branch_yaw = yaw_pt;
                        found      = true;
                    end
                end
            end
        end
    end

    % ---- PASS 2: accumulate ONLY the pixels of that one branch ----
    if found
        for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)
            for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)
                nrm = sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));
                if nrm >= MIN_RADIUS_CROWN && nrm <= MAX_RADIUS_CROWN
                    if frame(i,j) == 1
                        axis_x = (i-(COG_X-0.5));
                        axis_y = (j-(COG_Y-0.5));
                        yaw_pt = atan2(axis_x, axis_y);
                        dev_b = abs(atan2(sin(yaw_pt-branch_yaw), ...
                                          cos(yaw_pt-branch_yaw)));
                        if dev_b <= CLUSTER_HALF
                            pixel_count   = pixel_count + 1;
                            x_accumulator = x_accumulator + i;
                            y_accumulator = y_accumulator + j;
                        end
                    end
                end
            end
        end
    end

else
    % initial heading search (not yet locked on the lane): scan the full ring
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
end

% medium point of the lane = centroid of the selected branch
x_ref_temp = -(round(x_accumulator/pixel_count) - COG_X);
y_ref_temp =  (round(y_accumulator/pixel_count) - COG_Y);

if isnan(x_ref_temp)
    error_x_track = 0;
    flag_error_x  = false;
else
    flag_error_x  = true;
    error_x_track = x_ref_temp;
end

if isnan(y_ref_temp)
    error_y_track = 0;
    flag_error_y  = false;
else
    error_y_track = y_ref_temp;
    flag_error_y  = true;
end

flag_track = flag_error_x || flag_error_y;

% reset persistent accumulators for the next frame
pixel_count   = 0;
x_accumulator = 0;
y_accumulator = 0;

end
