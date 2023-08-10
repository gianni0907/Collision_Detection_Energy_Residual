function plot_data(q_des,dq_des,q_act,dq_act,t)
%Plot the desired joint space configuration and velocities vs the actual
%one executed by the robot

% first joint angle and velocity comparison
subplot(3,2,1)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad]","Interpreter","Latex"), title("$q_1$","Interpreter","Latex"), legend('desired','actual');
plot(t,q_des(:,1));
plot(t,q_act(:,1));
subplot(3,2,2)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad/s]","Interpreter","Latex"), title("$\dot{q}_1$","Interpreter","Latex"), legend('desired','actual');
plot(t,dq_des(:,1));
plot(t,dq_act(:,1));
% second joint angle and velocity comparison
subplot(3,2,3)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad]","Interpreter","Latex"), title("$q_2$","Interpreter","Latex"), legend('desired','actual');
plot(t,q_des(:,2));
plot(t,q_act(:,2));
subplot(3,2,4)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad/s]","Interpreter","Latex"), title("$\dot{q}_2$","Interpreter","Latex"), legend('desired','actual');
plot(t,dq_des(:,2));
plot(t,dq_act(:,2));
% third joint angle and velocity comparison
subplot(3,2,5)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad]","Interpreter","Latex"), title("$q_3$","Interpreter","Latex"), legend('desired','actual');
plot(t,q_des(:,3));
plot(t,q_act(:,3));
subplot(3,2,6)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad/s]","Interpreter","Latex"), title("$\dot{q}_3$","Interpreter","Latex"), legend('desired','actual');
plot(t,dq_des(:,3));
plot(t,dq_act(:,3));
end