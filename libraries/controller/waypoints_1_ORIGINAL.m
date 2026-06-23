function [error_y_track, error_x_track, orientation_track, flag_track] = ...
error_pixel_generator(frame, x_ref_prev, y_ref_prev, flag_track_prev, ...
MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y)
% The MATLAB function computes the error along x and y-axis between
% the drone COG (Center of Gravity) and the point of the track extract
% by using of a crown circular mask. This errors is later used by the Path
% Planner for the lane tracking
%
%  RECOVERED ORIGINAL CODE of the "waypoints 1" MATLAB Function block
%  (Image Processing System / Waypoints Follower). Restored from the
%  flightControlSystem.slx.r2019b backup after it was accidentally
%  overwritten with an MPC wrapper. The MPC controller belongs in the
%  "MPC block" inside Control System / Path Planning, NOT here.
%
% Inputs:
% - frame: binarized frame
% - x_ref_prev: previous value of the tracking point along the x-axis of
% the track line. It is used as a reference for tracking the path.
% - y_ref_prev: previous value of the tracking point along the y-axis of
% the track line. It is used as a reference for tracking the path.
% - flag_track_prev: previous value assumed by the flag variablle, i.e.,
% the previous state of the drone (over the track / not over the track).
%
% Outputs:
% - error_y_track: error between the CoG of the drone and a point of the
% lane in the forward direction along the y-axis. It is expressed in pixels.
% - error_x_track: error between the CoG of the drone and a point of the
% lane in the forward direction along the x-axis. It is expressed in pixels.
% - flag_track: it indicates if the drone is currently over the track (TRUE).
% - orientation_track: the heading angle of the track in the image plane.
%
% Parameters:
% - MIN_RADIUS_CROWN: minimum value of the radius of the crown mask [pixels].
% - MAX_RADIUS_CROWN: maximum value of the radius of the crown mask [pixels].
% - FOV: a portion of the crown used to detect the lane [radians].
% - COG_X: position of the drone CoG along x-axis of the reference frame.
% - COG_Y: position of the drone CoG along y-axis of the reference frame.

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

% the previous value of the heading angle of the lane
orientation_track = atan2(-x_ref_prev,y_ref_prev);
orientation_track = mod(orientation_track,2*pi);

% checking if the drone is over the lane
if flag_track_prev

    % the set of pixels inside the crown mask
    for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)

        for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)

            norm = sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));

            if norm >= MIN_RADIUS_CROWN && norm <= MAX_RADIUS_CROWN

                axis_x = (i-(COG_X-0.5));
                axis_y = (j-(COG_Y-0.5));
                yaw_point_temp = atan2(axis_x,axis_y);


                yaw_point_temp_2 = mod(yaw_point_temp,2*pi);

                % a portion of the crown mask depending on the FOV
                threshold_1 = mod((orientation_track+(pi/FOV)),2*pi);

                threshold_2 = mod((orientation_track-(pi/FOV)),2*pi);

                if threshold_1 < threshold_2

                    flag_threshold = ...
                        (yaw_point_temp_2 <= 2*pi  && yaw_point_temp_2 >= threshold_2) ...
                        || (yaw_point_temp_2 <= threshold_1  && yaw_point_temp_2 >= 0);

                else

                    flag_threshold = ...
                        yaw_point_temp_2 <= threshold_1  && yaw_point_temp_2 >= threshold_2;

                end

                if flag_threshold

                    if frame(i,j)==1

                        pixel_count = pixel_count + 1;
                        x_accumulator = x_accumulator + i;
                        y_accumulator = y_accumulator + j;

                    end

                end

            end

        end
    end

else
    % checking for the initial heading angle of the lane. It can be
    % different from the heading angle of the drone
    for i = (COG_X-MAX_RADIUS_CROWN):(COG_X+MAX_RADIUS_CROWN)

        for j = (COG_Y-MAX_RADIUS_CROWN):(COG_Y+MAX_RADIUS_CROWN)

            norm=sqrt(((i-(COG_X-0.5))^2)+((j-(COG_Y-0.5))^2));

            if norm>=MIN_RADIUS_CROWN && norm <= MAX_RADIUS_CROWN

                axis_x = (i-(COG_X-0.5));
                axis_y = (j-(COG_Y-0.5));

                if frame(i,j)==1
                    pixel_count=pixel_count +1;
                    x_accumulator=x_accumulator+i;
                    y_accumulator=y_accumulator+j;
                end
            end
        end
    end
end

% calculating the medium point of the lane as output of the mask
x_ref_temp = -(round(x_accumulator/pixel_count)-COG_X);
y_ref_temp = (round(y_accumulator/pixel_count)-COG_Y);

% when there is no track, the x_ref_temp and y_ref_temp variables are NaN.
% Below is the code for fix the issue.

if isnan(x_ref_temp)

    % if the computer vision algorithm does not detect the track, the
    % drone position does not change
    error_x_track = 0 ;
    flag_error_x = false;

else

    % if the computer vision algorithm detects the track, a new position
    % is computed for the tracking problem
    flag_error_x = true;
    error_x_track = x_ref_temp;

end

% As for the x coordinate
if isnan(y_ref_temp)

    error_y_track = 0 ;
    flag_error_y = false;

else

    error_y_track = y_ref_temp;
    flag_error_y=true ;

end

% checking if the drone is over the lane
flag_track = flag_error_x || flag_error_y;

% reset of the persistent variables
pixel_count = 0;
x_accumulator = 0;
y_accumulator = 0;

end
