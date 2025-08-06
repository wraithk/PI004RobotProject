function [wl, wr] = inverse_kinematics(u, q)
% Compute the left and right wheel velocities (wl, wr) required for the robot
% to achieve a forward speed u and angular speed q.

% The scale parameter and wheel track required to solve this are provided here.
% You can find these values in the robot simulator as well.
% In real-life, you would have to measure or calibrate them!
scale_parameter = 5.9e-3; % Real: 4.15e-3 %5.0-5.5
wheel_track = 0.15; % Real: 0.12

wl = u/scale_parameter - wheel_track*q/(2*scale_parameter);
wr = u/scale_parameter + wheel_track*q/(2*scale_parameter);

end