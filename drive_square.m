function drive_square(pb, sideLength)
% DRIVE_SQUARE send a sequence of commands to the pibot to make it drive in a square.
% pb is the pibot instance to send commands
% sideLength is the length of each side of the square to drive

drive_velocity = 0.2;
ang_velocity = 1;

side_length_time = sideLength/drive_velocity;
corner_time = (pi/2)/ang_velocity;

% Calculate wheel velocities for straight movement and rotation
[wls, wrs] = inverse_kinematics(drive_velocity, 0);
[wlc, wrc] = inverse_kinematics(0, ang_velocity);

% Drive 5 squares
for square = 1:5
    % Drive each side of the square
    for side = 1:4
        % Drive straight for one side
        pb.setVelocity([wls wrs], side_length_time);
        % Rotate at corner (90 degrees)
        pb.setVelocity([wlc wrc], corner_time);
    end
end

end