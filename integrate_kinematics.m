function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot

%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot

%   new_state is the state after integration, also in the form [x;y;theta]
if (ang_velocity < 1e-7)
    new_state(1) = state(1) + (lin_velocity/ang_velocity)*(sin(state(3)+dt*ang_velocity)-sin(state(3)));
    new_state(2) = state(2) + (lin_velocity/ang_velocity)*(cos(state(3)+dt*ang_velocity)-cos(state(3)));
    new_state(3) = state(3) + dt*ang_velocity;

else
    new_state(1) = state(1) + dt*cos(state(3))*lin_velocity;
    new_state(2) = state(2) + dt*sin(state(3))*lin_velocity;
    new_state(3) = state(3);
end