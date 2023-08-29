function plot_joint_space(q_des,dq_des,q,dq,dq_est,u,u_ext,t_salient,t)

time_axis = sort([t_salient, 0, 10]);

%% joint positions
f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

subplot(4,1,1)
hold on,grid on,ylabel("$\tau_{ext} \ [Nm]$","Interpreter","Latex");
plot(t,u_ext(1,:));
plot(t,u_ext(2,:));
plot(t,u_ext(3,:));
xlim([0 max(t)])
xline(t_salient,'--k');
xticks(time_axis)
legend('$\tau_{ext,1}$','$\tau_{ext,2}$','$\tau_{ext,3}$',"Interpreter","Latex", ...
    "Location","north","Orientation","Horizontal");

subplot(4,1,2)
hold on,grid on,ylabel("$[rad]$","Interpreter","Latex");
%title("Joint positions","Interpreter","Latex");
plot(t,q_des(1,:));
plot(t,q(1,:));
xlim([0 max(t)])
xline(t_salient,'--k');
xticks(time_axis)
legend('$q_{1,d}$','$q_{1}$',"Interpreter","Latex","Location","south","Orientation","Horizontal");
subplot(4,1,3)
hold on,grid on,ylabel("$[rad]$","Interpreter","Latex");
plot(t,q_des(2,:));
plot(t,q(2,:));
xlim([0 max(t)])
xline(t_salient,'--k');
xticks(time_axis)
legend('$q_{2,d}$','$q_{2}$',"Interpreter","Latex","Location","north","Orientation","Horizontal");
subplot(4,1,4)
hold on,grid on,xlabel("$t \ [s]$","Interpreter","Latex"),ylabel("$[rad]$","Interpreter","Latex");
plot(t,q_des(3,:));
plot(t,q(3,:));
xlim([0 max(t)])
xline(t_salient,'--k');
xticks(time_axis)
legend('$q_{3,d}$','$q_{3}$',"Interpreter","Latex","Location","south","Orientation","Horizontal");

%% joint velocities
% real vs estimated
f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

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
f=figure;
f.Position=[276.2000  414.6000  976.8000  320.8000];
set(0,'DefaultLineLineWidth',1)

hold on,grid on,xlabel("$t \ [s]$","Interpreter","Latex"),ylabel("$ \epsilon \ [rad/s]$","Interpreter","Latex");
%title("Joint velocity estimation error","Interpreter","Latex");
plot(t,dq(1,:)-dq_est(1,:));
plot(t,dq(2,:)-dq_est(2,:));
plot(t,dq(3,:)-dq_est(3,:));
xlim([0 max(t)])
xline(t_salient,'--k');
xticks(time_axis)
legend('$\epsilon_{1}$','$\epsilon_{2}$','$\epsilon_{3}$',"Interpreter","Latex");

% real vs desired
f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

subplot(3,1,1)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
title("Real vs desired joint velocities","Interpreter","Latex");
plot(t,dq(1,:));
plot(t,dq_des(1,:));
legend('$\dot{q}_{1}$','$\dot{q}_{1,d}$',"Interpreter","Latex");
subplot(3,1,2)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
plot(t,dq(2,:));
plot(t,dq_des(2,:));
legend('$\dot{q}_{2}$','$\dot{q}_{2,d}$',"Interpreter","Latex");
subplot(3,1,3)
hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[rad/s]$","Interpreter","Latex");
plot(t,dq(3,:));
plot(t,dq_des(3,:));
legend('$\dot{q}_{3}$','$\dot{q}_{3,d}$',"Interpreter","Latex");
%% joint torques
f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$[Nm]$","Interpreter","Latex");
title("Joint torques","Interpreter","Latex");
plot(t,u(1,:));
plot(t,u(2,:));
plot(t,u(3,:));
legend('$u_{1}$','$u_{2}$','$u_{3}$',"Interpreter","Latex");
%% joint velocity norm
f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

hold on,grid on,xlabel("$[s]$","Interpreter","Latex"),ylabel("$|| \dot{q} ||$","Interpreter","Latex");
title("Joint velocity norm","Interpreter","Latex");
plot(t,vecnorm(dq));
end