function drive_circle(pb, radius)
% DRIVE_CIRCLE send a sequence of commands to the pibot to make it drive in a circle.
% pb is the pibot instance to send commands
% radius is the radius of the circle to drive

velocity = 0.2;
circumference = 2*pi*radius;
driving_time = circumference/velocity;
angular_velocity = (2*pi)/driving_time;
[wl, wr] = inverse_kinematics(velocity, angular_velocity);

% Maximum time per command
max_time_per_command = 4;

if driving_time <= max_time_per_command
    % Single command if total time is 4 seconds or less
    pb.setVelocity([wl wr], driving_time);
else
    % Break into multiple commands of 4 seconds each
    num_full_segments = floor(driving_time / max_time_per_command);
    remaining_time = driving_time - (num_full_segments * max_time_per_command);
    
    % Send full 20-second segments
    for i = 1:num_full_segments
        pb.setVelocity([wl wr], max_time_per_command);
    end
    
    % Send remaining time segment if there's any left
    if remaining_time > 0
        pb.setVelocity([wl wr], remaining_time);
    end
end

end