% Returns thetas for new position after changing the position generated by
% the original thetas, with respect to the feeder frame.

function thetas_out = get_near_pos_ee(robot,thetas, x_inc, y_inc, z_inc,psi,theta,phi)
    theta_source = thetas;
    
    theta_1 = theta_source(1);
    theta_2 = theta_source(2);
    theta_3 = theta_source(3);
    theta_4 = theta_source(4);
    theta_5 = theta_source(5);
    d1 = 13.5;
    a2 = 40.5;
    a3 = 33;
    d4 = 13;
    d5 = 10.5;
    H_0_1 = [cos(theta_1), 0, sin(theta_1), 0;
             sin(theta_1), 0, -cos(theta_1), 0;
             0, 1, 0, d1;
             0, 0, 0, 1];
    H_1_2 = [cos(theta_2), sin(theta_2), 0, a2*cos(theta_2);
             sin(theta_2), -cos(theta_2), 0, a2*sin(theta_2);
             0, 0, -1, 0;
             0, 0, 0, 1];
    H_2_3 = [cos(theta_3), sin(theta_3), 0, a3*cos(theta_3);
             sin(theta_3), -cos(theta_3), 0, a3*sin(theta_3);
             0, 0, -1, 0;
             0, 0, 0, 1];
    H_3_4 = [cos(theta_4+pi/2), 0, sin(theta_4+pi/2), 0;
             sin(theta_4+pi/2), 0, -cos(theta_4+pi/2), 0;
             0, 1, 0, d4;
             0, 0, 0, 1];
    H_4_5 = [cos(theta_5), -sin(theta_5), 0, 0;
             sin(theta_5), cos(theta_5), 0, 0;
             0, 0, 1, d5;
             0, 0, 0, 1];
    
    r13 = sin(psi)*sin(phi)+cos(phi)*sin(theta)*cos(psi);
    r23 = -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
    r33 = cos(theta)*cos(psi);
    r32 = cos(theta)*sin(psi);
    r31 = -sin(theta);
    r12 = -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
    r22 = cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
    r11 = cos(phi)*cos(theta);
    r21 = sin(phi)*cos(theta);
    H = [r11,r12,r13,x_inc;r21,r22,r23,y_inc;r31,r32,r33,z_inc;0,0,0,1];
    
    H_0_5 = (H_0_1*H_1_2*H_2_3*H_3_4*H_4_5);
    new_frame = H_0_5*H;
    
    pos_new = [new_frame(1,4);new_frame(2,4);new_frame(3,4);get_orientation(new_frame)];
    disp(H_0_5);
    disp(new_frame);
    disp(pos_new);
    thetas_out = robot.inverse_kinematics_analytical(pos_new);
end