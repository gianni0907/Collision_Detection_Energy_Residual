function plot_joint_level(q_des,dq_des,q,dq,dq_est,u,t)
%Plot the desired joint space configuration and velocities vs the actual
%one executed by the robot

%% joint positions
figure;
subplot(3,1,1)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad]$","Interpreter","Latex");
title("Joint positions","Interpreter","Latex");
plot(t,q_des(1,:));
plot(t,q(1,:));
legend('$q_{1,des}$','$q_{1}$',"Interpreter","Latex");
subplot(3,1,2)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad]$","Interpreter","Latex");
plot(t,q_des(2,:));
plot(t,q(2,:));
legend('$q_{2,des}$','$q_{2}$',"Interpreter","Latex");
subplot(3,1,3)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad]$","Interpreter","Latex");
plot(t,q_des(3,:));
plot(t,q(3,:));
legend('$q_{3,des}$','$q_{3}$',"Interpreter","Latex");

%% joint velocities
% real vs estimated
figure;
subplot(3,1,1)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
title("Real vs estimated joint velocities","Interpreter","Latex");
plot(t,dq(1,:));
plot(t,dq_est(1,:));
legend('$\dot{q}_{1}$','$\hat{\dot{q}}_{1}$',"Interpreter","Latex");
subplot(3,1,2)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
plot(t,dq(2,:));
plot(t,dq_est(2,:));
legend('$\dot{q}_{2}$','$\hat{\dot{q}}_{2}$',"Interpreter","Latex");
subplot(3,1,3)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
plot(t,dq(3,:));
plot(t,dq_est(3,:));
legend('$\dot{q}_{3}$','$\hat{\dot{q}}_{3}$',"Interpreter","Latex");

% plot error=actual-estimated velocities
figure;
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
title("Estimation error","Interpreter","Latex");
plot(t,dq(1,:)-dq_est(1,:));
plot(t,dq(2,:)-dq_est(2,:));
plot(t,dq(3,:)-dq_est(3,:));
legend('$e_{1}$','$e_{2}$','$e_{3}$',"Interpreter","Latex");

% real vs desired
figure;
subplot(3,1,1)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
title("Real vs desired joint velocities","Interpreter","Latex");
plot(t,dq(1,:));
plot(t,dq_des(1,:));
legend('$\dot{q}_{1}$','$\dot{q}_{1,des}$',"Interpreter","Latex");
subplot(3,1,2)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
plot(t,dq(2,:));
plot(t,dq_des(2,:));
legend('$\dot{q}_{2}$','$\dot{q}_{2,des}$',"Interpreter","Latex");
subplot(3,1,3)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
plot(t,dq(3,:));
plot(t,dq_des(3,:));
legend('$\dot{q}_{3}$','$\dot{q}_{3,des}$',"Interpreter","Latex");
%% joint torques
figure;
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[Nm]$","Interpreter","Latex");
title("Joint torques","Interpreter","Latex");
plot(t,u(1,:));
plot(t,u(2,:));
plot(t,u(3,:));
legend('$u_{1}$','$u_{2}$','$u_{3}$',"Interpreter","Latex");

end