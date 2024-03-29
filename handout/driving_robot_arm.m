%% Robot arm
%{ 
Old params
d1 = 6.35;
a2 = 40.64;
a3 = 33.02;
d4 = 13.589;
d5 = 8.128;
%}

d1 = 13.5;
a2 = 40.5;
a3 = 33;
d4 = 13;
d5 = 10.5;

robot= Robot([0, pi/2, d1, 0; ...
              a2, pi, 0, 0; ...
              a3, pi, 0, 0; ...
              0, pi/2, d4, pi/2; ...
              0, 0, d5, 0], [0;0;0;0;0], [0;0;0;0;0]);
robot_dh= Robot([0, pi/2, d1, 0; ...
              a2, pi, 0, 0; ...
              a3, pi, 0, 0; ...
              0, pi/2, d4, pi/2; ...
              0, 0, d5, 0], [0;0;0;0;0], [0;0;0;0;0]);

%old jenga_base_thetas1: 0.9579    0.5849    1.1957   -0.9602    0.9579 

feeder_thetas = [-0.0036    0.7323    1.4531    0.0236    0.0015]';
feeder_thetas = get_near_pos_ee(robot_dh, feeder_thetas,1.4,-0.5,-2,0,0,0);
above_feeder_thetas = get_near_pos_ee(robot_dh, feeder_thetas,0,0,-8,0,0,0);

final_position_thetas = [0.4652;0.6359;2.0741;1.4723; -5.4990];
air_pos_thetas = [0.2755    1.2793    1.9223   -0.8603     1.0262]'; 
air_pos2_thetas = [0.7561    0.9410    1.7485   -0.6664    0.5822]';

feeder_pos = robot_dh.ee(feeder_thetas);
above_feeder_pos = robot_dh.ee(above_feeder_thetas);

% Block dimensions
%{
Length = 7.5cm
Width = 2.3cm
height = 1.4cm
%}
block_num = 3;
offset = 0;

layer_offset = 1.8;
above_offset = 12;
side_offset = -2.4;
vlayer_offset = 3.3;
side_z_offset = 0.3;
left_offset = 2;
right_offset = -2;
front_offset = -2;
back_offset = 2;

jenga_base_thetas = zeros(5,block_num*6);
above_jenga_base_thetas = zeros(5,block_num*6);
jenga_side_thetas = zeros(5,18);
above_jenga_side_thetas = zeros(5,18);
approach_jenga_base_thetas = zeros(5,block_num*6);
jenga_base_thetas(:,1) = [0.9579    0.5849    1.1957   -0.9602    0.9579]'+[0;0;0;0.02;0];
jenga_base_thetas(:,1) = get_near_pos_ee(robot_dh, jenga_base_thetas(:,1),0,0,0,0,0,0);
%jenga_base_thetas(:,1) = get_near_pos_ee(robot_dh, jenga_base_thetas(:,1),0.7,0,0,0,0.2,0);
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
jenga_base_thetas(:,index) = get_near_pos(robot_dh,jenga_base_thetas(:,1),right_offset,0,side_z_offset,0,0,0);
above_jenga_base_thetas(:,index) = get_near_pos(robot_dh, jenga_base_thetas(:,index),0,0,above_offset,0,0,0);
jenga_side_thetas(:,index) = get_near_pos(robot_dh, jenga_base_thetas(:,index),right_offset,0,side_z_offset,0,0,0);
above_jenga_side_thetas(:,index) = get_near_pos(robot_dh, jenga_side_thetas(:,index),0,0,above_offset,0,0,0);


for double_layer = 1:3
    % Vertical layer
    offset = offset + 3;
    % Offset from the middle block of the previous layer
    jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,offset-2),0,0,layer_offset,0,0,0)+[0;0;0;0;pi/2];
    above_jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,1+offset),0,0,above_offset,0,0,0);
    jenga_side_thetas(:,1+offset) = jenga_base_thetas(:,1+offset);
    above_jenga_side_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,1+offset),0,0,above_offset,0,0,0);

    % Front block
    index = 2;
    jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),0,front_offset,side_z_offset,0,0,0);
    above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
    jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,front_offset,side_z_offset,0,0,0);
    above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

    % Back block
    index = 3;
    jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),0,back_offset,side_z_offset,0,0,0);
    above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
    jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,back_offset,side_z_offset,0,0,0);
    above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

    if double_layer ~= 3
        % Horizontal layer
        offset = offset + 3;
        % Offset from the first block
        jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,1),0,0,layer_offset*double_layer,0,0,0);
        above_jenga_base_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,1+offset),0,0,above_offset,0,0,0);
        jenga_side_thetas(:,1+offset) = jenga_base_thetas(:,1+offset);
        above_jenga_side_thetas(:,1+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,1+offset),0,0,above_offset,0,0,0);

        index = 2;
        jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),left_offset,0,side_z_offset,0,0,0);
        above_jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),0,0,above_offset,0,0,0);
        jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_base_thetas(:,index+offset),left_offset,0,side_z_offset,0,0,0);
        above_jenga_side_thetas(:,index+offset) = get_near_pos(robot_dh, jenga_side_thetas(:,index+offset),0,0,above_offset,0,0,0);

        index = 3;
        jenga_base_thetas(:,index+offset) = get_near_pos(robot_dh,jenga_base_thetas(:,index-1+offset),right_offset,0,side_z_offset,0,0,0);
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
    jenga_base_pos(:,index) = robot.ee(jenga_base_thetas(:,index));
    above_jenga_base_pos(:,index) = robot.ee(above_jenga_base_thetas(:,index));
    jenga_side_pos(:,index) = robot.ee(jenga_side_thetas(:,index));
    above_jenga_side_pos(:,index) = robot.ee(above_jenga_side_thetas(:,index));
end

pos = zeros(5,18);
for index = 1:18
    pos1 = (robot_dh.ee(feeder_thetas)+jenga_base_pos(:,index))/2;
    pos1 = [pos1(1:3);jenga_base_pos(4:6,index)];
    pos(:,index) = robot_dh.inverse_kinematics_analytical(pos1,jenga_base_pos(:,index));
end

%%

resolution = 50;

start = 1;
%{
for index = start:3
    pos = (robot.ee(feeder_thetas)+jenga_base_pos(:,index))/2;
    pos = [pos(1:3);jenga_base_pos(4:6,index)];
    pos = robot.inverse_kinematics_analytical(pos,jenga_base_pos(:,index));
    trajectory1 = [linear_workspace_trajectory(robot_dh,feeder_thetas,above_feeder_pos,resolution), ...
                  trajectory_spline([above_feeder_thetas,pos],[0,1],100), ...     
                  linear_workspace_trajectory(robot_dh,pos,approach_jenga_base_pos(:,index),90), ...
                  linear_workspace_trajectory(robot_dh,approach_jenga_base_thetas(:,index),above_jenga_base_pos(:,index),placing_resolution), ...
                  linear_workspace_trajectory(robot_dh,above_jenga_base_thetas(:,index),jenga_base_pos(:,index),placing_resolution)];

    trajectory2 = [linear_workspace_trajectory(robot_dh,jenga_base_thetas(:,index),above_jenga_base_pos(:,index),placing_resolution), ...
                   linear_workspace_trajectory(robot_dh,above_jenga_base_thetas(:,index),approach_jenga_base_pos(:,index),resolution*1.5), ... 
                   linear_workspace_trajectory(robot_dh,approach_jenga_base_thetas(:,index),robot_dh.ee(pos),placing_resolution), ...
                   trajectory_spline([pos,above_feeder_thetas],[0,1],100), ...
                   linear_workspace_trajectory(robot_dh,above_feeder_thetas, feeder_pos,resolution)];
    if index == start
        trajectory = trajectory1;
    else
        trajectory = [trajectory, trajectory1];
    end
end
%}

%trajectory = trajectory_spline([far_above_feeder_thetas,air_pos3_thetas,far_above_jenga_base_thetas1],[0,2,4],100);
%trajectory = linear_workspace_trajectory(robot,above_jenga_base_thetas(:,7),jenga_base_pos(:,7),resolution);
start = 1;
placing_resolution = 30;
resolution = 30;
for index = start:3
    trajectory1 = [linear_workspace_trajectory(robot_dh,feeder_thetas,above_feeder_pos,30), ...
                  trajectory_spline([above_feeder_thetas,pos(:,index)],[0,0.8],50), ... 
                  linear_workspace_trajectory(robot_dh,pos(:,index),above_jenga_side_pos(:,index),resolution*2+10), ...
                  linear_workspace_trajectory(robot_dh,above_jenga_side_thetas(:,index),jenga_side_pos(:,index),placing_resolution), ...
                  linear_workspace_trajectory(robot_dh,jenga_side_thetas(:,index),jenga_base_pos(:,index),placing_resolution)];
    
    trajectory2 = [linear_workspace_trajectory(robot_dh,jenga_base_thetas(:,index),above_jenga_base_pos(:,index),placing_resolution), ...
                   linear_workspace_trajectory(robot_dh,above_jenga_base_thetas(:,index),robot_dh.ee(pos(:,index)),resolution+30), ... 
                   trajectory_spline([pos(:,index),above_feeder_thetas],[0,0.8],100), ...
                   linear_workspace_trajectory(robot_dh,above_feeder_thetas, feeder_pos,30)];
    
    if index == start
        trajectory = [trajectory1, trajectory2];
    else
        trajectory = [trajectory, trajectory1, trajectory2];
    end
end


start_feeder_thetas = get_near_pos_ee(robot_dh, feeder_thetas,3.55,-0.45,-2,0,0,0);
above_start_feeder_thetas = get_near_pos_ee(robot_dh, start_feeder_thetas,0,0,-8,0,0,0);
start_feeder_pos = robot_dh.ee(start_feeder_thetas);

resolution = 5;
frequency = 5;
index = 1;
initial_thetas = air_pos2_thetas;
%trajectory = [trajectory_spline([initial_thetas air_pos_thetas above_start_feeder_thetas], [0, 2, 4], frequency), ...
%              linear_workspace_trajectory(robot_dh,above_start_feeder_thetas, start_feeder_pos,resolution), ...
%              ];
%disp(trajectory);

%%
n = size(trajectory,2);
x = zeros(n,1);
y = zeros(n,1);
z = zeros(n,1);
phi = zeros(n,1);
theta = zeros(n,1);
psi = zeros(n,1);
for i = 1:n
    %disp(joint_angles(:,i));
    ee = robot.ee(trajectory(:,i));
    x(i) = ee(1);
    y(i) = ee(2);
    z(i) = ee(3);
    psi(i) = ee(4);
    theta(i) = ee(5);
    phi(i) = ee(6);
end

figure();
%plot3(x, y, z, 'k-', 'LineWidth', 1);
plot3(x,y,z,'-o','Color','b','MarkerSize',5);
title('Plot of end effector position over a sample run.');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
axis equal;
%command_trajectory(robot, trajectory, frequency);


