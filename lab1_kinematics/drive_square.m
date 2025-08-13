function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.
% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive

drive_velocity = 0.2;
ang_velocity = 2;

side_length_time = sideLength/drive_velocity;
corner_time = (pi/2)/ang_velocity;
max_time_per_command = 4;

% Calculate wheel velocities for straight movement and rotation
[wls, wrs] = inverse_kinematics(drive_velocity, 0);
[wlc, wrc] = inverse_kinematics(0, ang_velocity);

% Drive each side of the square
for side = 1:4
    if side_length_time <= max_time_per_command
        pb.setVelocity([wls wrs], side_length_time);
    else
        num_full_segments = floor(side_length_time / max_time_per_command);
        remaining_time = side_length_time - (num_full_segments * max_time_per_command);
        
        % Send full 4-second segments
        for i = 1:num_full_segments
            pb.setVelocity([wls wrs], max_time_per_command);
        end

        if remaining_time > 0
        pb.setVelocity([wls wrs], remaining_time);
        end

    end
    % Send remaining time segment if there's any left
   
    % Rotate at corner (90 degrees)
    pb.setVelocity([wlc wrc], corner_time);
    disp('here')
end
end




