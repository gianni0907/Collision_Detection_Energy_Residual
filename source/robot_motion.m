function robot_motion(robot,q,p0,p1,c,r_c,T,T2,t)
% showing the motion of the robot

figure
show(robot,homeConfiguration(robot),'Frames','off');
cla
hold on

plot3(p0(1),p0(2),p0(3),'.','MarkerSize',18);
t_1=0:0.05:T;
t_2=0:0.05:T2/2;
plot3(c(1)*ones(size(t_1,2),1),c(2)+r_c*cos(2*pi/T*t_1),c(3)+r_c*sin(2*pi/T*t_1),'LineWidth',1.2)
plot3(p1(1),p1(2),p1(3),'.','MarkerSize',18);
plot3(p2(1),p2(2),p2(3),'.','MarkerSize',18);
plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],'LineWidth',1.2);
plot3(c(1)+r_c*cos(2*pi/T2*t_2),c(2)*ones(size(t_2,2),1),c(3)-r_c*sin(2*pi/T2*t_2),'LineWidth',1.2)

framesPerSecond = 20;
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