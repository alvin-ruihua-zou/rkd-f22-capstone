%{
Parameters for hw robot
[0, pi/2, 56.05, 0; ...
              330.3, 0, 0, 0; ...
              254.1, 0, 0, 0; ...
              0, pi/2, 121.5, 0; ...
              0, 0, 213.75, 0]

[0, pi/2, 56.05, 0; ...
              330.3, pi, 0, 0; ...
              254.1, pi, 0, 0; ...
              0, pi/2, 121.5, 0; ...
              0, 0, 213.75, 0]
%}



d1 = 6.35;
a2 = 40.64;
a3 = 33.02;
d4 = 13.589;
d5 = 8.128;

robot= Robot([0, pi/2, d1, 0; ...
              a2, pi, 0, 0; ...
              a3, pi, 0, 0; ...
              0, pi/2, d4, pi/2; ...
              0, 0, d5, 0], [0;0;0;0;0], [0;0;0;0;0]);


% Failed thetas: [1.7;2;1;3.2;3.7], [1;2;1;1.2;1.7], [1.7;2;-1;3.2;4], [1.7;1;3;3.2;-1.5]
% acos is imaginary for theta_3 [5.7;-0.1;-3;1.2;4]

% Failed positions: [49.6006;-12.5539;8.1879;-1.8570;0.1435;0.8738]
thetas_fk = [3;1;3;1;1];
[frames, valid] = robot.fk(thetas_fk);

if valid
    
    disp(robot.ee(thetas_fk));
    frame_ee = frames(:,:,end);
    x = frame_ee(1,4);
    y = frame_ee(2,4);
    z = frame_ee(3,4);
    psi = atan2(frame_ee(3,2), frame_ee(3,3));
    theta = -asin(frame_ee(3,1));
    phi = atan2(frame_ee(2,1), frame_ee(1,1));
    
    thetas = robot.inverse_kinematics_analytical([x;y;z;psi;theta;phi]);
else
    disp("invalid frame assignment");
end

