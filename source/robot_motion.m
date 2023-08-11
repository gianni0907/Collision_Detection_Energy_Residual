function robot_motion(DHTable,l,r,q)
%showing the motion of the robot
dhparams = [DHTable(:,2) DHTable(:,1) DHTable(:,3:4)];
robot = rigidBodyTree;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;

addBody(robot,body2,'body1');
addBody(robot,body3,'body2');
mat1 = [elem_rot_mat('x',pi/2) [0 -l(1)/2 0]';
        zeros(1,3)              1];
mat2 = [elem_rot_mat('y',pi/2)  [-l(2)/2 0 0]';
        zeros(1,3)               1];
mat3 = [elem_rot_mat('y',pi/2)  [-l(3)/2 0 0]';
        zeros(1,3)               1];
cylinder1 = collisionCylinder(r(1),l(1));
cylinder2 = collisionCylinder(r(2),l(2));
cylinder3 = collisionCylinder(r(3),l(3));
cylinder1.Pose = mat1;
cylinder2.Pose = mat2;
cylinder3.Pose = mat3;
addCollision(robot.Bodies{1},cylinder1);
addCollision(robot.Bodies{2},cylinder2);
addCollision(robot.Bodies{3},cylinder3);
% show(robot,'Collisions','off');
%% animate
framesPerSecond = 15;
r = rateControl(framesPerSecond);
config = homeConfiguration(robot);

for i = 1:size(q,1)
    config(1).JointPosition = q(i,1);
    config(2).JointPosition = q(i,2);
    config(3).JointPosition = q(i,3);
    % On the right subplot, preserve all previous
    % drawings, on the left subplot, only keep the
    % most recent drawing. Note the 'Parent' parameter
    % selects in which axis the robot is drawn
    show(robot, config, 'PreservePlot', false, 'FastUpdate', 1,'Collisions','on');
    hold on
    drawnow
    waitfor(r);
end
end