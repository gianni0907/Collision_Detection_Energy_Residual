function plot_data(q_des,dq_des,q_act,dq_act,u,t)
%Plot the desired joint space configuration and velocities vs the actual
%one executed by the robot

% first joint
subplot(3,3,1)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad]","Interpreter","Latex"), title("$q_1$","Interpreter","Latex");
plot(t,q_des(:,1));
plot(t,q_act(:,1));
legend('desired','actual');
subplot(3,3,2)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad/s]","Interpreter","Latex"), title("$\dot{q}_1$","Interpreter","Latex");
plot(t,dq_des(:,1));
plot(t,dq_act(:,1));
legend('desired','actual');
subplot(3,3,3)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[Nm]","Interpreter","Latex"), title("$u_1$","Interpreter","Latex");
plot(t,u(:,1));
% second joint angle and velocity comparison
subplot(3,3,4)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad]","Interpreter","Latex"), title("$q_2$","Interpreter","Latex");
plot(t,q_des(:,2));
plot(t,q_act(:,2));
legend('desired','actual');
subplot(3,3,5)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad/s]","Interpreter","Latex"), title("$\dot{q}_2$","Interpreter","Latex");
plot(t,dq_des(:,2));
plot(t,dq_act(:,2));
legend('desired','actual');
subplot(3,3,6)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[Nm]","Interpreter","Latex"), title("$u_2$","Interpreter","Latex");
plot(t,u(:,2));
% third joint angle and velocity comparison
subplot(3,3,7)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad]","Interpreter","Latex"), title("$q_3$","Interpreter","Latex");
plot(t,q_des(:,3));
plot(t,q_act(:,3));
legend('desired','actual');
subplot(3,3,8)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[rad/s]","Interpreter","Latex"), title("$\dot{q}_3$","Interpreter","Latex");
plot(t,dq_des(:,3));
plot(t,dq_act(:,3));
legend('desired','actual');
subplot(3,3,9)
hold on, xlabel("[s]","Interpreter","Latex"),ylabel("[Nm]","Interpreter","Latex"), title("$u_3$","Interpreter","Latex");
plot(t,u(:,3));
end