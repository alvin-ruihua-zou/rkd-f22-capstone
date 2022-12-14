function [] = pick_place_sample()
% pick_place_sample
%
% pick and place example code; picks up an object at position "A" on
% the table and moves it to position "B".
%% First, we define a couple of helper functions.  You can break these out into
%% separate files if you wish.
% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle 
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
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
% Convenience function to use to hide the internal logic of starting the suction
function [] = pick(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 1;
  suction_cup.set(suction_cmd);
end
% Convenience function to use to hide the internal logic of stopping the suction
function [] = place(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 0;
  suction_cup.set(suction_cmd);
end
% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);
% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(1);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
gains = load('jenga_gains.mat');
gains.jenga_gains.positionKp = [1 2 2 2 1];
gains.jenga_gains.positionKi = [0 0 0 0 0];
gains.jenga_gains.positionKd = [0 0.01 0.01 0.01 0.01];
gains.jenga_gains.positionFF = [0.0 0.05 0.05 0.0 0.0];

HebiUtils.sendWithRetry(robot, 'gains', gains.jenga_gains);

%% Connect to gripper, and initialize some settings properly
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);
warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');
%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)
%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));
%% command frequency, in Hz

% Create a set of "approach" angles that let us have a slow "final approach" to
% the actual pick and place location.  This can increase accuracy and reduce
% issues where straight-line-configuration-space trajectories make the end
% effector hit the table
%offset = 1 for vertical
%

d1 = 13.5;
a2 = 40.5;
a3 = 33;
d4 = 13;
d5 = 10.5;

robot_dh= Robot([0, pi/2, d1, 0; ...
              a2, pi, 0, 0; ...
              a3, pi, 0, 0; ...
              0, pi/2, d4, pi/2; ...
              0, 0, d5, 0], [0;0;0;0;0], [0;0;0;0;0]);




feeder_thetas = [-0.0036    0.7323    1.4531    0.0236    0.0015]';
feeder_thetas = get_near_pos_ee(robot_dh, feeder_thetas,3.55,-0.4,-2.1,0,0,0);
start_feeder_thetas = get_near_pos_ee(robot_dh, feeder_thetas,3.55,-0.45,-2,0,0,0);
above_feeder_thetas = get_near_pos_ee(robot_dh, feeder_thetas,0,0,-8,0,0,0);
above_start_feeder_thetas = get_near_pos_ee(robot_dh, start_feeder_thetas,0,0,-8,0,0,0);

final_position_thetas = [0.4652;0.6359;2.0741;1.4723; -5.4990];
air_pos_thetas = [0.2755    1.2793    1.9223   -0.8603     1.0262]'; 
air_pos2_thetas = [0.7561    0.9410    1.7485   -0.6664    0.5822]';

feeder_pos = robot_dh.ee(feeder_thetas);
start_feeder_pos = robot_dh.ee(start_feeder_thetas);
above_feeder_pos = robot_dh.ee(above_feeder_thetas);

block_num = 3;
offset = 0;
layer_offset = 2.65;
above_offset = 6;
side_offset = -2.8;
vlayer_offset = -2.6;
v_layer_side_offset = 2.7;
side_z_offset = 0.5;
left_offset = 2;
right_offset = -2.5;
front_offset = -2.5;
back_offset = 2;

jenga_base_thetas = zeros(5,block_num*6);
above_jenga_base_thetas = zeros(5,block_num*6);
jenga_side_thetas = zeros(5,18);
jenga_base_thetas(:,1) = [0.9579    0.5849    1.1957   -0.9602    0.9579]';
jenga_base_thetas(:,1) = get_near_pos(robot_dh, jenga_base_thetas(:,1),0,1.5,2.2,0,0,0);
above_jenga_base_thetas(:,1) = get_near_pos(robot_dh, jenga_base_thetas(:,1),0,0,above_offset,0,0,0);
jenga_side_thetas(:,1) = jenga_base_thetas(:,1);
above_jenga_side_thetas(:,1) = get_near_pos(robot_dh, jenga_side_thetas(:,1),0,0,above_offset,0,0,0);

% Block on left
index = 2;
jenga_base_thetas(:,index) = get_near_pos(robot_dh,jenga_base_thetas(:,1),left_offset,0,side_z_offset,0,0,0);
above_jenga_base_thetas(:,index) = get_near_pos(robot_dh, jenga_base_thetas(:,index),0,0,above_offset,0,0,0);
jenga_side_thetas(:,index) = get_near_pos(robot_dh, jenga_base_thetas(:,index),left_offset,0,side_z_offset,0,0,0);
above_jenga_side_thetas(:,index) = get_near_pos(robot_dh, jenga_side_thetas(:,index),0,0,above_offset,0,0,0);

% Block on right
index = 3;
jenga_base_thetas(:,index) = get_near_pos(robot_dh,jenga_base_thetas(:,1),right_offset+0.6,0,side_z_offset,0,0,0);
above_jenga_base_thetas(:,index) = get_near_pos(robot_dh, jenga_base_thetas(:,index),0,0,above_offset,0,0,0);
jenga_side_thetas(:,index) = get_near_pos(robot_dh, jenga_base_thetas(:,index),right_offset,0,side_z_offset,0,0,0);
above_jenga_side_thetas(:,index) = get_near_pos(robot_dh, jenga_side_thetas(:,index),0,0,above_offset,0,0,0);

back_z_offset = 0;
middle_z_offset = 0;
middle_y = 0;
for double_layer = 1:3
    % Vertical layer
    offset = offset + 3;
    if double_layer == 3
        middle_z_offset = 0;
        layer_offset = layer_offset -0.1 ;
        middle_y = 0.6;
    end
    % Offset from the middle block of the previous layer
    jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,offset-2),0,middle_y,layer_offset+middle_z_offset,0,0,0)+[0;0;0;0;pi/2];
    above_jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,1+offset),0,0,above_offset,0,0,0);
    jenga_side_thetas(:,1+offset) = jenga_base_thetas(:,1+offset);
    above_jenga_side_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,1+offset),0,0,above_offset,0,0,0);

    if double_layer == 2
        layer_offset = layer_offset + 0.5;
        front_offset = front_offset + 0;
        back_offset = back_offset + 0.3;
        back_z_offset = back_z_offset + 0.6;
        left_offset = left_offset - 0.7;
        right_offset = right_offset -0.1;
    end

    
    % Front block
    index = 2;
    
    jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),0,front_offset+1.3,side_z_offset+0.2,0,0,0);
    above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
    jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,front_offset,side_z_offset,0,0,0);
    above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

    % Back block
    index = 3;
    jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),0,back_offset+0.7,side_z_offset+back_z_offset,0,0,0);
    above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
    jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,back_offset,side_z_offset,0,0,0);
    above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

    if double_layer ~= 3
        % Horizontal layer
        offset = offset + 3;
        % Offset from the first block
        jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,1),-1,0,layer_offset*double_layer+0.4,0,0,0);
        above_jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,1+offset),0,0,above_offset,0,0,0);
        jenga_side_thetas(:,1+offset) = jenga_base_thetas(:,1+offset);
        above_jenga_side_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,1+offset),0,0,above_offset,0,0,0);

        index = 2;
        jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),left_offset,0,side_z_offset+0.6,0,0,0);
        above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
        jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),left_offset,0,side_z_offset+0.4,0,0,0);
        above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

        index = 3;
        jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),right_offset-1,0,side_z_offset,0,0,0);
        above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
        jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),right_offset,0,side_z_offset,0,0,0);
        above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

    end
end

jenga_base_pos = zeros(6,size(jenga_base_thetas,2));
above_jenga_base_pos = zeros(6,size(jenga_base_thetas,2));
jenga_side_pos = zeros(6,size(jenga_base_thetas,2));
above_jenga_side_pos = zeros(6,size(jenga_base_thetas,2));
for index = 1:size(jenga_base_pos,2)
    jenga_base_pos(:,index) = robot_dh.ee(jenga_base_thetas(:,index));
    above_jenga_base_pos(:,index) = robot_dh.ee(above_jenga_base_thetas(:,index));
    jenga_side_pos(:,index) = robot_dh.ee(jenga_side_thetas(:,index));
    above_jenga_side_pos(:,index) = robot_dh.ee(above_jenga_side_thetas(:,index));
end

pos = zeros(5,18);
for index = 1:18
    pos1 = (robot_dh.ee(feeder_thetas)+jenga_base_pos(:,index))/2;
    pos1 = [pos1(1:3);jenga_base_pos(4:6,index)];
    pos(:,index) = robot_dh.inverse_kinematics_analytical(pos1,jenga_base_pos(:,index));
end

disp("finished initial");

%% Moves the robot from the initial position to the first waypoint over 4
%% seconds.  We break this into 3 seconds to make most of the motion, and 1 for
%% the final approach.
resolution = 50;
placing_resolution = 80;
frequency = 100;

trajectory = [trajectory_spline([initial_thetas air_pos_thetas above_feeder_thetas], [0, 2, 4], frequency), ...
              linear_workspace_trajectory(robot_dh,above_feeder_thetas, feeder_pos,resolution), ...
              ];
command_trajectory(robot, trajectory, frequency);
%% Pick up the object at position 1.  We pause to let the robot stabilize before
%% moving. NOTE: If you pause more than the length of the command lifetime you
%% set above, then the robot "goes limp" because the previous position and/or
%% velocity and torque commands "expire".

for index = 1:18
    % Assume ee is at feeder. Move block to base and then move back to
    % feeder.

    pick(gripper);
    pause(0.1);
    
    
    trajectory = [linear_workspace_trajectory(robot_dh,feeder_thetas,above_feeder_pos,30), ...
                  trajectory_spline([above_feeder_thetas,pos(:,index)],[0,0.8],100), ... 
                  linear_workspace_trajectory(robot_dh,pos(:,index),above_jenga_side_pos(:,index),resolution+30), ...
                  linear_workspace_trajectory(robot_dh,above_jenga_side_thetas(:,index),jenga_side_pos(:,index),placing_resolution), ...
                  linear_workspace_trajectory(robot_dh,jenga_side_thetas(:,index),jenga_base_pos(:,index),20)];
    command_trajectory(robot, trajectory, frequency);
   
    
    pause(0.1);
    place(gripper);
    pause(0.15);

    
    trajectory = [linear_workspace_trajectory(robot_dh,jenga_base_thetas(:,index),above_jenga_base_pos(:,index),placing_resolution), ...
                   linear_workspace_trajectory(robot_dh,above_jenga_base_thetas(:,index),robot_dh.ee(pos(:,index)),resolution+30), ... 
                   trajectory_spline([pos(:,index),above_feeder_thetas],[0,0.8],100), ...
                   linear_workspace_trajectory(robot_dh,above_feeder_thetas, feeder_pos,30)];
    command_trajectory(robot, trajectory, frequency);
   
end

%% Move to rest position
trajectory = [linear_workspace_trajectory(robot_dh,feeder_thetas,above_feeder_pos,resolution), ...
              trajectory_spline([above_feeder_thetas air_pos_thetas final_position_thetas], [0,1.5,3], frequency)];
command_trajectory(robot, trajectory, frequency);




%% Stop logging, and plot results
robot.stopLog();
hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));
% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.effort, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');
end
