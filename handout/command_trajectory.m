function [] = command_trajectory(robot, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();

  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);

  % Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)' * frequency;
    robot.set(cmd);

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end

  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  robot.set(cmd);
end