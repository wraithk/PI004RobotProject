function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.
% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive

drive_velocity = 0.2;
ang_velocity = 2;

side_length_time = sideLength/drive_velocity;
corner_time = (pi/2)/ang_velocity;


% Calculate wheel velocities for straight movement and rotation
[wls, wrs] = inverse_kinematics(drive_velocity, 0);
[wlc, wrc] = inverse_kinematics(0, ang_velocity);

% Drive 5 squares
% Drive each side of the square
for side = 1:4
    % Drive straight for one side
    disp(side_length_time);
    pb.setVelocity([wls wrs], side_length_time/4);
    pb.setVelocity([wls wrs], side_length_time/4);
    pb.setVelocity([wls wrs], side_length_time/4);
    pb.setVelocity([wls wrs], side_length_time/4);
    % Rotate at corner (90 degrees)
    pb.setVelocity([wlc wrc], corner_time);
end
end


% Auto divides into a number of defined time segments


%{
function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.
% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive

drive_velocity = 0.25;
ang_velocity = 1.5;
max_time_per_command = 4;

% total times for one side and one corner
side_time = sideLength/drive_velocity;
turn_time = (pi/2)/ang_velocity;

% wheel speeds for straight / rotation
[wls, wrs] = inverse_kinematics(drive_velocity, 0);
[wlc, wrc] = inverse_kinematics(0, ang_velocity);

for side = 1:4
    driving_time = side_time;
    if driving_time <= max_time_per_command
        pb.setVelocity([wls wrs], driving_time);
    else
        num_full_segments = floor(driving_time / max_time_per_command);
        remaining_time  = driving_time - num_full_segments*max_time_per_command;
        for i = 1:num_full_segments
            pb.setVelocity([wls wrs], max_time_per_command);
        end
        if remaining_time > 0
            pb.setVelocity([wls wrs], remaining_time);
        end
    end

    % Rotate 90 at the corner 
    rotating_time = turn_time;
    if rotating_time <= max_time_per_command
        pb.setVelocity([wlc wrc], rotating_time);
    else
        num_full_segments = floor(rotating_time / max_time_per_command);
        remaining_time  = rotating_time - num_full_segments*max_time_per_command;
        for i = 1:n_full
            pb.setVelocity([wlc wrc], max_time_per_command);
        end
        if remaining_time > 0
            pb.setVelocity([wlc wrc], remaining_time);
        end
    end
end
end

%}