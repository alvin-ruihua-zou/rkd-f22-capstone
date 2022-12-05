dhparams = [0, pi/2, 56.05/2000, 0; ...
              330.3/2000, pi, 0, 0; ...
              254.1/2000, pi, 0, 0; ...
              0, pi/2, 121.5/2000, 0; ...
              0, 0, 213.75/2000, 0];

robot = rigidBodyTree;

bodies = cell(5,1);
joints = cell(5,1);
for i = 1:5
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base") 
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end


%showdetails(robot)

%show(robot);
%axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])
%axis off

aik = analyticalInverseKinematics(robot);
showdetails(aik)