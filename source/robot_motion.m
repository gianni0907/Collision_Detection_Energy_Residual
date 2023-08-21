function robot_motion(robot,q,t)
%showing the motion of the robot

framesPerSecond = floor(size(t,2)/20);
r = rateControl(framesPerSecond);
config = homeConfiguration(robot);
for i = 1:size(q,2)
    config(1).JointPosition = q(1,i);
    config(2).JointPosition = q(2,i);
    config(3).JointPosition = q(3,i);
    show(robot, config, 'PreservePlot', false, 'FastUpdate', 1,'Collisions','on');
    hold on
    title('$t = '+compose("%.2f",t(i))+' s$','Interpreter','Latex');
    drawnow
    waitfor(r);
end
end