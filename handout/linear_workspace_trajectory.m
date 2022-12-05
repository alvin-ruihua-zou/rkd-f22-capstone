function [ trajectory ] = linear_workspace_trajectory(robot, start_theta, goal_pos, num_points)
% linear_workspace_trajectory
%
%   Returns a matrix of joint angles, where each column represents a single
%   timestamp. This matrix is length(start_theta) rows and num_points columns.
%
%   Using these joint angles, the end effector linearly interpolates from the
%   location given configuration start_theta to goal_pos, in the workspace.
%
%   'robot' is the Robot object to use for any required IK calculations.
%
%   'start_theta' is a column vector of joint angles
%
%   'goal_pos' is a 2 or 3 element column vector representing the goal position
%   (or position and orientation) of the end effector.
%
%   'num_points' is the number of columns in the resulting trajectory.

% --------------- BEGIN STUDENT SECTION ----------------------------------

trajectory = zeros(size(start_theta,1), num_points);

% We know the first column:
trajectory(:, 1) = start_theta;

% HINT: it may be useful to first calculate the desired workspace trajectory to
% reference in the loop below

start_pos = robot.ee(start_theta);
start_pos = start_pos(1:size(goal_pos,1));
waypoints_margin = (goal_pos - start_pos)/num_points; 

% Find the rest:
for col = 2:num_points
    
    trajectory(:,col) = robot.inverse_kinematics_analytical(start_pos+waypoints_margin*(col-1),start_pos+waypoints_margin*(col-2));

    %% Fill in trajectory(:,col) here. HINT: use trajectory(:,col-1) to help!

% --------------- END STUDENT SECTION ------------------------------------
end

end
