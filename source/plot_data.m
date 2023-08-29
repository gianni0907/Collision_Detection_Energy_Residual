function plot_data(p,dp,p_des,dp_des,f_ext,u_ext,sigma,r,residual_threshold,P_ext,t_salient,t)

time_axis = sort([t_salient, 0, 10]);

f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

subplot(3,1,1)
hold on, grid on, ylabel("$r \ [Nm]$","Interpreter","Latex");
plot(t,r(1,:)),
plot(t,r(2,:));
plot(t,r(3,:));
xline(t_salient,'--k');
xticks(time_axis)
legend("$r_{1}$","$r_{2}$","$r_{3}$","Interpreter","Latex");
title("Momentum residual","Interpreter","Latex");

subplot(3,1,2)
hold on, grid on, ylabel("$\tau_{ext} \ [Nm]$","Interpreter","Latex");
plot(t,u_ext(1,:)),
plot(t,u_ext(2,:));
plot(t,u_ext(3,:));
xline(t_salient,'--k');
xticks(time_axis)
legend("$\tau_{ext,1}$","$\tau_{ext,2}$","$\tau_{ext,3}$","Interpreter","Latex");
title("$\tau_{ext}$","Interpreter","Latex")

subplot(3,1,3)
hold on, grid on, ylabel("$[Nm]$","Interpreter","Latex");
plot(t,u_ext(1,:)-r(1,:)),
plot(t,u_ext(2,:)-r(2,:));
plot(t,u_ext(3,:)-r(3,:));
xline(t_salient,'--k');
xticks(time_axis)
legend("$e_{1}$","$e_{2}$","$e_{3}$","Interpreter","Latex");
title("$\tau_{ext}-r$","Interpreter","Latex")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

subplot(2,1,1)
hold on, grid on, ylabel("$p \ [m]$","Interpreter","Latex");
plot(t,p(1,:),'b'),
plot(t,p(2,:),'r');
plot(t,p(3,:),'m');
plot(t,p_des(1,:),'b--'),
plot(t,p_des(2,:),'r--');
plot(t,p_des(3,:),'m--');
xline(t_salient,'--k');
xticks(time_axis)
legend("$p^{ee}_x$","$p^{ee}_y$","$p^{ee}_z$","$p^{ee}_{x,des}$","$p^{ee}_{y,des}$","$p^{ee}_{z,des}$", ...
    "Interpreter","Latex");
title("Cartesian quantities $p^{ee}$ and $\dot{p}^{ee}$","Interpreter","Latex");

subplot(2,1,2)
hold on, grid on, ylabel("$\dot{p} \ [m/s]$","Interpreter","Latex");
plot(t,dp(1,:),'b'), hold on;
plot(t,dp(2,:),'r');
plot(t,dp(3,:),'m');
plot(t,dp_des(1,:),'b--'),
plot(t,dp_des(2,:),'r--');
plot(t,dp_des(3,:),'m--');
xline(t_salient,'--k');
xticks(time_axis)
legend("$\dot{p}_x$","$\dot{p}_y$","$\dot{p}_z$",...
        "$\dot{p}_{x,des}$","$\dot{p}_{y,des}$","$\dot{p}_{z,des}$","Interpreter","Latex");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.WindowState='max';
set(0,'DefaultLineLineWidth',1)

subplot(5,1,1)
hold on, grid on, ylabel("$p^{ee} \ [m]$","Interpreter","Latex");
plot(t,p(1,:)),
plot(t,p(2,:));
plot(t,p(3,:));
xline(t_salient,'--k');
legend("$p^{ee}_x$","$p^{ee}_y$","$p^{ee}_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(5,1,2)
hold on, grid on, ylabel("$\dot{p}^{ee} \ [m/s]$","Interpreter","Latex");
plot(t,dp(1,:)), hold on;
plot(t,dp(2,:));
plot(t,dp(3,:));
xline(t_salient,'--k');
legend("$\dot{p}^{ee}_x$","$\dot{p}^{ee}_y$","$\dot{p}^{ee}_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(5,1,3)
hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex");
plot(t,f_ext(1,:)), hold on;
plot(t,f_ext(2,:));
plot(t,f_ext(3,:));
xline(t_salient,'--k');
legend("$f_{ext,x}$","$f_{ext,y}$","$f_{ext,z}$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(5,1,4)
hold on, plot(t,sigma)
xline(t_salient,'--k');
yline([residual_threshold -residual_threshold],'--r')
for i=1:size(sigma,2)-1
    if abs(sigma(i+1))>residual_threshold && abs(sigma(i))<residual_threshold
        if norm(f_ext(:,i+1))>0
            plot(t(i+1),sigma(i+1),'*g');
        else
            plot(t(i+1),sigma(i+1),'*k');
        end
    end
end
grid on, ylabel("$\sigma \ [W]$","Interpreter","Latex");
xticks(time_axis)
xlim([0 max(t)])

subplot(5,1,5)
plot(t,P_ext)
xline(t_salient,'--k');
grid on, ylabel("$P_{ext} \ [W]$","Interpreter","Latex"), xlabel("$t \ [s]$","Interpreter","Latex");
xticks(time_axis)
box off
xlim([0 max(t)])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.Position=[276.2,77,988.8,658.4];
set(0,'DefaultLineLineWidth',1)

subplot(3,1,1)
hold on, grid on, ylabel("$ p^{ee}_d \ [m]$","Interpreter","Latex");
plot(t,p_des(1,:)),
plot(t,p_des(2,:));
plot(t,p_des(3,:));
xline(t_salient,'--k');
xticks(time_axis)
legend("$p^{ee}_{x,d}$","$p^{ee}_{y,d}$","$p^{ee}_{z,d}$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xlim([0 max(t)])

subplot(3,1,2)
hold on, grid on, ylabel("$\dot{p}^{ee}_d \ [m/s]$","Interpreter","Latex");
plot(t,dp_des(1,:)),
plot(t,dp_des(2,:));
plot(t,dp_des(3,:));
xline(t_salient,'--k');
xticks(time_axis)
legend("$\dot{p}^{ee}_{x,d}$","$\dot{p}^{ee}_{y,d}$","$\dot{p}^{ee}_{z,d}$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xlim([0 max(t)])

subplot(3,1,3)
hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex"),xlabel("$t \ [s]$","Interpreter","Latex");
plot(t,f_ext(1,:)), hold on;
plot(t,f_ext(2,:));
plot(t,f_ext(3,:));
xline(t_salient,'--k');
legend("$f_{ext,x}$","$f_{ext,y}$","$f_{ext,z}$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.WindowState='max';
set(0,'DefaultLineLineWidth',1)

subplot(4,1,1)
hold on, grid on, ylabel("$p^{ee} \ [m]$","Interpreter","Latex");
plot(t,p(1,:)),
plot(t,p(2,:));
plot(t,p(3,:));
xline(t_salient,'--k');
legend("$p^{ee}_x$","$p^{ee}_y$","$p^{ee}_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(4,1,2)
hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex");
plot(t,f_ext(1,:)), hold on;
plot(t,f_ext(2,:));
plot(t,f_ext(3,:));
xline(t_salient,'--k');
legend("$f_{ext,x}$","$f_{ext,y}$","$f_{ext,z}$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(4,1,3)
hold on, plot(t,sigma)
xline(t_salient,'--k');
yline([residual_threshold -residual_threshold],'--r')
for i=1:size(sigma,2)-1
    if abs(sigma(i+1))>residual_threshold && abs(sigma(i))<residual_threshold
        if norm(f_ext(:,i+1))>0
            plot(t(i+1),sigma(i+1),'*g');
        else
            plot(t(i+1),sigma(i+1),'*k');
        end
    end
end
grid on, ylabel("$\sigma \ [W]$","Interpreter","Latex");
xticks(time_axis)
xlim([0 max(t)])

subplot(4,1,4)
plot(t,P_ext)
xline(t_salient,'--k');
grid on, ylabel("$P_{ext} \ [W]$","Interpreter","Latex"), xlabel("$t \ [s]$","Interpreter","Latex");
xticks(time_axis)
box off
xlim([0 max(t)])
end