function plot_data(p,dp,p_des,dp_des,f_ext,r,residual_threshold,P_ext,t_salient,t)

time_axis = sort([t_salient, 0, 5, 10, 15]);

figure
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
legend("$p_x$","$p_y$","$p_z$","$p_{x,des}$","$p_{y,des}$","$p_{z,des}$","Interpreter","Latex");

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

f=figure;
f.WindowState='max';

subplot(5,1,1)
hold on, grid on, ylabel("$p \ [m]$","Interpreter","Latex");
plot(t,p(1,:)),
plot(t,p(2,:));
plot(t,p(3,:));
xline(t_salient,'--k');
legend("$p_x$","$p_y$","$p_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)

subplot(5,1,2)
hold on, grid on, ylabel("$\dot{p} \ [m/s]$","Interpreter","Latex");
plot(t,dp(1,:)), hold on;
plot(t,dp(2,:));
plot(t,dp(3,:));
xline(t_salient,'--k');
legend("$\dot{p}_x$","$\dot{p}_y$","$\dot{p}_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)

subplot(5,1,3)
hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex");
plot(t,f_ext(1,:)), hold on;
plot(t,f_ext(2,:));
plot(t,f_ext(3,:));
xline(t_salient,'--k');
legend("$f_x$","$f_y$","$f_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)

subplot(5,1,4)
hold on, plot(t,r)
xline(t_salient,'--k');
yline([residual_threshold -residual_threshold],'--r')
for i=1:size(r,2)-1
    if abs(r(i+1))>residual_threshold && abs(r(i))<residual_threshold
        plot(t(i+1),r(i+1),'*r')
    end
end
grid on, ylabel("$r \ [W]$","Interpreter","Latex");
xticks(time_axis)

subplot(5,1,5)
plot(t,P_ext)
xline(t_salient,'--k');
grid on, ylabel("$P_{ext} \ [W]$","Interpreter","Latex"), xlabel("$t \ [s]$","Interpreter","Latex");
xticks(time_axis)

end