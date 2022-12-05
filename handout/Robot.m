classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_masses
        joint_masses
        dh_parameters
    end
    
    methods
        %% Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(dh_parameters, link_masses, joint_masses)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(dh_parameters, 2) ~= 4
                error('Invalid dh_parameters: Should be a dof x 4 matrix, is %dx%d.', size(dh_parameters, 1), size(dh_parameters, 2));
            end
            
            if size(link_masses, 2) ~= 1
                error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
                error('Invalid joint_masses: Should be a column vector.');
            end
            
            robot.dof = size(dh_parameters, 1);
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of degrees of freedom. Did you forget the base joint?');
            end
            
            robot.dh_parameters = dh_parameters;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
        end
        
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        
        %% Foward Kinematics        
        function [frames, valid] = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 4x4 matrix frames(:,:,i).
            
            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 4x4 matrix frames(:,:,end).
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            alpha = zeros(size(robot.dh_parameters,1),1);
            alpha(:,1) = robot.dh_parameters(:,2);
            a = zeros(size(robot.dh_parameters,1),1);
            a(:,1) = robot.dh_parameters(:,1);
            d = zeros(size(robot.dh_parameters,1),1);
            d(:,1) = robot.dh_parameters(:,3);
            thetas(1) = thetas(1)+robot.dh_parameters(1,4);
            H_0_1 = [cos(thetas(1)), -sin(thetas(1))*cos(alpha(1)), sin(thetas(1))*sin(alpha(1)), a(1)*cos(thetas(1));
                     sin(thetas(1)), cos(thetas(1))*cos(alpha(1)), -cos(thetas(1))*sin(alpha(1)), a(1)*sin(thetas(1));
                     0, sin(alpha(1)), cos(alpha(1)), d(1);
                     0, 0, 0, 1];

            frames(:,:,1) = H_0_1;
            endpoints = zeros(robot.dof+1,3);
            endpoints(1,:) = [frames(1,4,1), frames(2,4,1), frames(3,4,1)];
            % Calculate frames for the joints
            for i = 2:n
                thetas(i) = thetas(i)+robot.dh_parameters(i,4);
                H_prev_i = [cos(thetas(i)), -sin(thetas(i))*cos(alpha(i)), sin(thetas(i))*sin(alpha(i)), a(i)*cos(thetas(i));
                            sin(thetas(i)), cos(thetas(i))*cos(alpha(i)), -cos(thetas(i))*sin(alpha(i)), a(i)*sin(thetas(i));
                            0, sin(alpha(i)), cos(alpha(i)), d(i);
                            0, 0, 0, 1];
                H_0_i = frames(:,:,i-1)*H_prev_i;
                frames(:,:,i) = H_0_i;
                endpoints(i,:)  = [frames(1,4,i), frames(2,4,i), frames(3,4,i)];
                
                
            end
            endpoints(n+1,:) = [0,0,0];
            
            %v1 = checkIntersect(endpoints(1:2,:), endpoints(2:3,:));
            v2 = checkIntersect([endpoints(1,:);endpoints(3,:)], [endpoints(2,:);endpoints(4,:)]);
            v3 = checkIntersect([endpoints(1,:);endpoints(4,:)], [endpoints(2,:);endpoints(5,:)]);
            %v4 = checkIntersect(endpoints(2:3,:), endpoints(3:4,:));
            v5 = checkIntersect([endpoints(2,:);endpoints(4,:)], [endpoints(3,:);endpoints(5,:)]);
            %v6 = checkIntersect(endpoints(3:4,:), endpoints(4:5,:));
            %v7 = checkIntersect([endpoints(6,:);endpoints(1,:)], endpoints(1:2,:));
            v8 = checkIntersect([endpoints(6,:);endpoints(2,:)], [endpoints(1,:);endpoints(3,:)]);
            v9 = checkIntersect([endpoints(6,:);endpoints(3,:)], [endpoints(1,:);endpoints(4,:)]);
            v10 = checkIntersect([endpoints(6,:);endpoints(4,:)], [endpoints(1,:);endpoints(5,:)]);
            
            % If theta = pi then the arm will self intersect
            v1 = (thetas(3) == pi || thetas(3) == -pi);
            v4 = (thetas(4) == pi || thetas(4) == -pi);

            % Check if any of the arms go below z = 0 (under the table)
            v6 = (endpoints(1,3) <= 0 || endpoints(2,3) <= 0 || endpoints(3,3) <= 0 || ...
                  endpoints(4,3) <= 0 || endpoints(5,3) <= 0);
            
            if v1 || v2 || v3 || v4 || v5 || v6 || v8 || v9 || v10
                %disp(endpoints);
                %disp("invalid");
                %disp([v1,v2,v3,v4,v5,v6,v8,v9,v10]);
                %if v6
                 %   disp(endpoints);
                %end
                valid = false;
            else
                valid = true;
            end


            


            

            %{
            H_1_2 = [cos(theta(2)), -sin(theta(2))*cos(alpha(2)), sin(theta(2))*sin(alpha(2)), a(2)*cos(theta(2));
                     sin(theta(2)), cos(theta(2))*cos(alpha(2)), -cos(theta(2))*sin(alpha(2)), a(2)*sin(theta(2));
                     0, sin(alpha(2)), cos(alpha(2)), d(2);
                     0, 0, 0, 1];
            H_2_3 = [cos(theta(3)), -sin(theta(3))*cos(alpha(3)), sin(theta(3))*sin(alpha(3)), a(3)*cos(theta(3));
                     sin(theta(3)), cos(theta(3))*cos(alpha(3)), -cos(theta(3))*sin(alpha(3)), a(3)*sin(theta(3));
                     0, sin(alpha(3)), cos(alpha(3)), d(3);
                     0, 0, 0, 1];
            H_3_4 = [cos(theta(4)), -sin(theta(4))*cos(alpha(4)), sin(theta(4))*sin(alpha(4)), a(4)*cos(theta(4));
                     sin(theta(4)), cos(theta(4))*cos(alpha(4)), -cos(theta(4))*sin(alpha(4)), a(4)*sin(theta(4));
                     0, sin(alpha(4)), cos(alpha(4)), d(4);
                     0, 0, 0, 1];
            H_4_5 = [cos(theta(5)), -sin(theta(5))*cos(alpha(5)), sin(theta(5))*sin(alpha(5)), a(5)*cos(theta(5));
                     sin(theta(5)), cos(theta(5))*cos(alpha(5)), -cos(theta(5))*sin(alpha(5)), a(5)*sin(theta(5));
                     0, sin(alpha(5)), cos(alpha(5)), d(5);
                     0, 0, 0, 1];
            %}

            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        % Shorthand for returning the forward kinematics.
        function [fk, valid] = fk(robot, thetas)
            [fk, valid] = robot.forward_kinematics(thetas);
        end


        

        
        % Returns [x; y; z; psi; theta; phi] for the end effector given a
        % set of joint angles. Remember that psi is the roll, theta is the
        % pitch, and phi is the yaw angle.
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
            
            % Extract the components of the end_effector position and
            % orientation.
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            x = H_0_ee(1,4);
            y = H_0_ee(2,4);
            z = H_0_ee(3,4);
            theta = atan2(-H_0_ee(3,1), sqrt(H_0_ee(3,2)^2+H_0_ee(3,3)^2));
            phi = atan2(H_0_ee(2,1),H_0_ee(1,1));
            psi = atan2(H_0_ee(3,2), H_0_ee(3,3));


            % --------------- END STUDENT SECTION ------------------------------------
            
            % Pack them up nicely.
            ee = [x; y; z; psi; theta; phi];
        end
        
        % Shorthand for returning the end effector position and orientation.
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        %% Jacobians
        
        function jacobians = jacobians_numerical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            epsilon = 0.001;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
           

            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function jacobians = jacobians_analytical(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
                error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end
            
            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % TODO build up the jacobian using the analytical
            % convention from lecture

            frames = robot.fk(thetas);
            H_0_1 = frames(:,:,end);
            H_0_2 = frames(:,:,end);
            H_0_3 = frames(:,:,end);
            H_0_4 = frames(:,:,end);
            H_0_5 = frames(:,:,end);
            z1 = H_0_1(1:3,3);
            z2 = H_0_2(1:3,3);
            z3 = H_0_3(1:3,3);
            z4 = H_0_4(1:3,3);
            z5 = H_0_5(1:3,3);
            o1 = H_0_1(1:3,4);
            o2 = H_0_2(1:3,4);
            o3 = H_0_3(1:3,4);
            o4 = H_0_4(1:3,4);
            o5 = H_0_5(1:3,4);
            jacobians = [[z1*(o5-o1);z1], [z2*(o5-o2);z2], [z3*(o5-o3);z3], [z4*(o5-o4);z4], [z5*(o5-o5);z5]];


            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        
        %% Inverse Kinematics
        
        function thetas = inverse_kinematics_graddescent(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            thetas = initial_thetas;
            
            % Step size for gradient update
            step_size = 0.0000005;
            
            % Once the norm (magnitude) of the computed gradient is smaller than
            % this value, we stop the optimization
            stopping_condition = 0.00005;
            
            % Also, limit to a maximum number of iterations.
            max_iter = 50000;
            num_iter = 0;
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            % Run gradient descent optimization
            while (num_iter < max_iter)
                
                % Compute the gradient for either an [x;y;z] goal or an
                % [x; y; z; psi; theta; phi] goal, using the current value of 'thetas'.
                % TODO fill in the gradient of the squared distance cost function
                % HINT use the answer for theory question 2, the
                % 'robot.end_effector' function, and the 'robot.jacobians'
                % function to help solve this problem
            
                
                % Update 'thetas'
                % TODO
                
                % Check stopping condition, and return if it is met.
                % TODO
                
                num_iter = num_iter + 1;
            end
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function cost = cost_function(robot, thetas, goal_position)
            % Cost function for fmincon
            current_pose = robot.ee(thetas);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            % --------------- END STUDENT SECTION ------------------------------------
            
        end
        
        function thetas = inverse_kinematics_numopt(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function. Using built in optimization (fmincon)
            
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
                error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            end
            
            % Allocate a variable for the joint angles during the optimization;
            % begin with the initial condition
            
            fun = @(thetas)robot.cost_function(thetas, goal_position);
            
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = inverse_kinematics_analytical(robot, goal_position, prev_position)
            % Returns the joint angles using an analytical approach to
            % inverse kinematics
            % Note: Kinematics Decoupling might be very useful for this
            % question
            
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            %if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
            %    error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
            %end
            
            thetas = zeros(robot.dof, 1);
            
            % --------------- BEGIN STUDENT SECTION ----------------------------------
            
            
            
            x_ee = real(goal_position(1));
            y_ee = real(goal_position(2));
            z_ee = real(goal_position(3));
            psi = real(goal_position(4));
            theta = real(goal_position(5));
            phi = real(goal_position(6));
            px = x_ee;
            py = y_ee;
            pz = z_ee;
            r13 = sin(psi)*sin(phi)+cos(phi)*sin(theta)*cos(psi);
            r23 = -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
            r33 = cos(theta)*cos(psi);
            r32 = cos(theta)*sin(psi);
            r31 = -sin(theta);
            r12 = -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
            r22 = cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
            r11 = cos(phi)*cos(theta);
            r21 = sin(phi)*cos(theta);
            

            a = zeros(size(robot.dh_parameters,1),1);
            a(:,1) = robot.dh_parameters(:,1);
            d = zeros(size(robot.dh_parameters,1),1);
            d(:,1) = robot.dh_parameters(:,3);


            
            
            % Theta_1 and theta_5 have 2 solution
            theta_1 = atan2(y_ee,x_ee)+atan2(real(d(4)/sqrt(x_ee^2+y_ee^2)),real(sqrt(1-d(4)^2/(x_ee^2+y_ee^2))));
            theta_1_2 = atan2(y_ee,x_ee)+atan2(real(d(4)/sqrt(x_ee^2+y_ee^2)),real(-sqrt(1-d(4)^2/(x_ee^2+y_ee^2))));
            
            theta_5 = atan2(r11*sin(theta_1)-r21*cos(theta_1), r12*sin(theta_1)-r22*cos(theta_1));
            theta_5_2 = atan2(r11*sin(theta_1_2)-r21*cos(theta_1_2), r12*sin(theta_1_2)-r22*cos(theta_1_2));

            

            %x_c = (x_ee - d(5)*r13+d(4)*r12);
            %y_c = (y_ee - d(5)*r23+d(4)*r22);
            z_c = (z_ee - d(5)*r33);
            s = z_c - d(1);
            
            x_c_1 = x_ee-d(4)*sin(theta_1)- d(5)*r13;
            y_c_1 = y_ee+d(4)*cos(theta_1)- d(5)*r23;
            x_c_2 = x_ee-d(4)*sin(theta_1_2)- d(5)*r13;
            y_c_2 = y_ee+d(4)*cos(theta_1_2)- d(5)*r23;
            s_squared = real((z_c-d(1))^2);
            r_squared = real(x_c_1^2+(y_c_1)^2);
            r_squared_2 = real((x_c_2)^2+(y_c_2)^2);
            % Sign is always positive for r_1
            sign_1 = 1;
            %sign_2 = -1;
            
            if s < 0 || y_c_2 < 0 
                sign_2 = -1;
            else
                sign_2 = 1;
            end
            

            % Adjust the input into acos for theta_3
            in_theta_3_1 = (r_squared+s_squared-a(2)^2-a(3)^2)/(2*a(2)*a(3));
            if (in_theta_3_1) > 1
                in_theta_3_1 = in_theta_3_1 - 1;
            elseif (in_theta_3_1) < -1
                in_theta_3_1 = 1+ in_theta_3_1;
            end
            
            in_theta_3_2 = (r_squared_2+s_squared-a(2)^2-a(3)^2)/(2*a(2)*a(3));
            if (in_theta_3_2) > 1
                in_theta_3_2 = in_theta_3_2 - 1;
            elseif (in_theta_3_2) < -1
                in_theta_3_2 = 1+ in_theta_3_2;
            end

            % Theta_3 has 4 solutions
            % Using theta_1
            theta_3_1 = acos(in_theta_3_1);
            theta_3_2 = -theta_3_1;
            % Using theta_1_2
            
            theta_3_3 = acos(in_theta_3_2);
            theta_3_4 = -theta_3_3;

            % Thus, theta_2 and theta_4 will also have four solutions 
            theta_2_2 = atan2(s,sign_1*sqrt(r_squared))-atan2((a(3)*sin(theta_3_1)),(a(2)+a(3)*cos(theta_3_1)));
            theta_2_1 = atan2(s,sign_1*sqrt(r_squared))-atan2((a(3)*sin(theta_3_2)),(a(2)+a(3)*cos(theta_3_2)));
            theta_2_22 = atan2(s,-sign_1*sqrt(r_squared))-atan2((a(3)*sin(theta_3_1)),(a(2)+a(3)*cos(theta_3_1)));
            theta_2_12 = atan2(s,-sign_1*sqrt(r_squared))-atan2((a(3)*sin(theta_3_2)),(a(2)+a(3)*cos(theta_3_2)));
            theta_2_4 = atan2(s,sign_2*sqrt(r_squared_2))-atan2((a(3)*sin(theta_3_3)),(a(2)+a(3)*cos(theta_3_3)));
            theta_2_3 = atan2(s,sign_2*sqrt(r_squared_2))-atan2((a(3)*sin(theta_3_4)),(a(2)+a(3)*cos(theta_3_4)));
            t1 = atan2(s,-sign_2*sqrt(r_squared_2))-atan2((a(3)*sin(theta_3_3)),(a(2)+a(3)*cos(theta_3_3)));
            t2 = atan2(s,-sign_2*sqrt(r_squared_2))-atan2((a(3)*sin(theta_3_4)),(a(2)+a(3)*cos(theta_3_4)));
            %disp([theta_2_2, theta_2_1, theta_2_4, theta_2_3;theta_2_22, theta_2_12, t1, t2]);
            
            
            theta_4_1 = atan2(cos(theta_2_1 - theta_3_1)*(r13*cos(theta_1) + r23*sin(theta_1)) + r33*sin(theta_2_1 - theta_3_1), ...
                              -(r33*cos(theta_2_1 - theta_3_1) - sin(theta_2_1 - theta_3_1)*(r13*cos(theta_1) + r23*sin(theta_1))));
            theta_4_2 = atan2(cos(theta_2_2 - theta_3_2)*(r13*cos(theta_1) + r23*sin(theta_1)) + r33*sin(theta_2_2 - theta_3_2), ...
                              -(r33*cos(theta_2_2 - theta_3_2) - sin(theta_2_2 - theta_3_2)*(r13*cos(theta_1) + r23*sin(theta_1))));
            theta_4_3 = atan2(cos(theta_2_3 - theta_3_3)*(r13*cos(theta_1_2) + r23*sin(theta_1_2)) + r33*sin(theta_2_3 - theta_3_3), ...
                              -(r33*cos(theta_2_3 - theta_3_3) - sin(theta_2_3 - theta_3_3)*(r13*cos(theta_1_2) + r23*sin(theta_1_2))));
            theta_4_4 = atan2(cos(theta_2_4 - theta_3_4)*(r13*cos(theta_1_2) + r23*sin(theta_1_2)) + r33*sin(theta_2_4 - theta_3_4), ...
                              -(r33*cos(theta_2_4 - theta_3_4) - sin(theta_2_4 - theta_3_4)*(r13*cos(theta_1_2) + r23*sin(theta_1_2))));
            
            % Add the offset from dh
            theta_4_1 = theta_4_1 - robot.dh_parameters(4,4);
            theta_4_2 = theta_4_2 - robot.dh_parameters(4,4);
            theta_4_3 = theta_4_3 - robot.dh_parameters(4,4);
            theta_4_4 = theta_4_4 - robot.dh_parameters(4,4);
           
            % Run fk on the different solutions for validation
            
            [fk_1, valid_1] = robot.fk([theta_1;theta_2_1;theta_3_1;theta_4_1;theta_5]);
            
            [fk_2, valid_2] = robot.fk([theta_1;theta_2_2;theta_3_2;theta_4_2;theta_5]);
           
            [fk_5, valid_5] = robot.fk([theta_1_2;theta_2_3;theta_3_3;theta_4_3;theta_5_2]);
            
            [fk_6, valid_6] = robot.fk([theta_1_2;theta_2_4;theta_3_4;theta_4_4;theta_5_2]);
            
           
            H_0_5 = [r11,r12,r13,x_ee;r21,r22,r23,y_ee;r31,r32,r33,z_ee;0,0,0,1];

            

            fk_1_valid = false;
            fk_2_valid = false;
            
            fk_5_valid = false;
            fk_6_valid = false;
            
            thetas_fk1 = [theta_1;theta_2_1;theta_3_1;theta_4_1;theta_5];
            thetas_fk2 = [theta_1;theta_2_2;theta_3_2;theta_4_2;theta_5];

            thetas_fk5 = [theta_1_2;theta_2_3;theta_3_3;theta_4_3;theta_5_2];
            thetas_fk6 = [theta_1_2;theta_2_4;theta_3_4;theta_4_4;theta_5_2];

            if matrix_iseq(fk_1(:,:,end),H_0_5) && valid_1
                disp("fk_1 correct");
                %disp(fk_1(:,:,end));
                thetas(2) = theta_2_1;
                thetas(3) = theta_3_1;
                thetas(4) = theta_4_1;
                thetas(1) = theta_1;
                thetas(5) = theta_5;
                fk_1_valid = true;
                fk_1_thetas = [theta_1;theta_2_1;theta_3_1;theta_4_1;theta_5];
                
            end
        
            if matrix_iseq(fk_2(:,:,end),H_0_5) && valid_2
                disp("fk_2 correct");
                %disp(fk_2(:,:,end));
                thetas(2) = theta_2_2;
                thetas(3) = theta_3_2;
                thetas(4) = theta_4_2;
                thetas(1) = theta_1;
                thetas(5) = theta_5;
                fk_2_valid = true;
                fk_2_thetas = [theta_1;theta_2_2;theta_3_2;theta_4_2;theta_5];
                
            end

            

            if matrix_iseq(fk_5(:,:,end),H_0_5) && valid_5
                disp("fk_5 correct");
                thetas(1) = theta_1_2;
                thetas(2) = theta_2_3;
                thetas(3) = theta_3_3;
                thetas(4) = theta_4_3;
                
                thetas(5) = theta_5_2;
                fk_5_valid = true;
                fk_5_thetas = [theta_1_2;theta_2_3;theta_3_3;theta_4_3;theta_5_2];
                
            end
            if matrix_iseq(fk_6(:,:,end),H_0_5) && valid_6
                disp("fk_6 correct");
                thetas(1) = theta_1_2;
                thetas(2) = theta_2_4;
                thetas(3) = theta_3_4;
                thetas(4) = theta_4_4;
                thetas(5) = theta_5_2;
                fk_6_valid = true;
                fk_6_thetas = [theta_1_2;theta_2_4;theta_3_4;theta_4_4;theta_5_2];
                
            end
            
            if exist('prev_position','var')
                disp("exist");
                closest_fk = [0;0;0;0;0];
                distance = 100000;
                dist_1 = abs(theta_1-prev_position(1))+abs(theta_2_1-prev_position(2))...
                   +abs(theta_3_1-prev_position(3))+abs(theta_4_1-prev_position(4))+abs(theta_5-prev_position(5));
                dist_2 = abs(theta_1-prev_position(1))+abs(theta_2_2-prev_position(2))...
                   +abs(theta_3_2-prev_position(3))+abs(theta_4_2-prev_position(4))+abs(theta_5-prev_position(5));
                dist_5 = abs(theta_1_2-prev_position(1))+abs(theta_2_3-prev_position(2))...
                   +abs(theta_3_3-prev_position(3))+abs(theta_4_3-prev_position(4))+abs(theta_5_2-prev_position(5));
                dist_6 = abs(theta_1_2-prev_position(1))+abs(theta_2_4-prev_position(2))...
                   +abs(theta_3_4-prev_position(3))+abs(theta_4_4-prev_position(4))+abs(theta_5_2-prev_position(5));
                if fk_1_valid && dist_1 < distance
                    closest_fk = fk_1_thetas;
                    distance = dist_1;
                end
                if fk_2_valid && dist_2 < distance
                    closest_fk = fk_2_thetas;
                    distance = dist_2;
                end
                if fk_5_valid && dist_5 < distance
                    closest_fk = fk_5_thetas;
                    distance = dist_5;
                end
                if fk_1_valid && dist_6 < distance
                    closest_fk = thetas_fk6;
                end
                thetas = closest_fk;


            end

            if thetas(1) < -pi
                thetas(1) = 2*pi+thetas(1);
            end
            if thetas(2) < -pi
                thetas(2) = 2*pi+thetas(2);
            end
            if thetas(3) < -pi
                thetas(3) = 2*pi+thetas(3);
            end
            if thetas(4) < -pi
                thetas(4) = 2*pi+thetas(4);
            end
            if thetas(5) < -pi
                thetas(5) = 2*pi+thetas(5);
            end


            
            if ~fk_1_valid && ~fk_2_valid && ...
               ~fk_5_valid && ~fk_6_valid 
                disp("no solution found");
                disp([x_ee,y_ee,z_ee,psi, theta, phi]);
                if phi == 0 || theta == 0 || psi == 0
                    disp("singularity!");
                    disp([psi, theta, phi]);
                end
                thetas(2) = theta_2_1;
                thetas(3) = theta_3_1;
                thetas(4) = theta_4_1;
                fk1 = robot.fk(thetas_fk1);
                fk2 = robot.fk(thetas_fk2);
                fk5 = robot.fk(thetas_fk5);
                fk6 = robot.fk(thetas_fk6);
                disp(H_0_5);
                disp([fk1(:,:,end);fk2(:,:,end);fk5(:,:,end);fk6(:,:,end)]);
                
            end
            
            
            disp([thetas_fk1,thetas_fk2,thetas_fk5,thetas_fk6]);
            
            % --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = endpoints_to_waypoints(robot, endpoints)
            thetas(:, 1) = robot.inverse_kinematics_analytical(endpoints(:, 1));
            for theta_col=2:size(endpoints, 2)
                thetas(:, theta_col) = robot.inverse_kinematics_analytical(endpoints(:, theta_col),endpoints(:, theta_col-1));
            end
        end
        
    end
end