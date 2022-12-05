function thetas_out = get_near_pos(robot, thetas,x_inc, y_inc, z_inc,psi,theta,phi)
    pos = robot.ee(thetas);
    pos_new = pos + [x_inc;y_inc;z_inc;psi;theta;phi];
    thetas_out = robot.inverse_kinematics_analytical(pos_new);
end