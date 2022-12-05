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