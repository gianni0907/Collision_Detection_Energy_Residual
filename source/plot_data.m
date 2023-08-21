function plot_data(p,dp,p_des,dp_des,f_ext,r,residual_threshold,P_ext,t_salient,t)

f=figure;
f.WindowState='max';
% first joint
% subplot(5,1,1)
% hold on, grid on, ylabel("$p \ [m]$","Interpreter","Latex");
% plot(t,p(1,:)),
% plot(t,p(2,:));
% plot(t,p(3,:));
% plot(t,p_des(1,:)),
% plot(t,p_des(2,:));
% plot(t,p_des(3,:));
% legend("$p_x$","$p_y$","$p_z$","$p_{x,des}$","$p_{y,des}$","$p_{z,des}$","Interpreter","Latex");
% 
% subplot(5,1,2)
% hold on, grid on, ylabel("$\dot{p} \ [m/s]$","Interpreter","Latex");
% plot(t,dp(1,:)), hold on;
% plot(t,dp(2,:));
% plot(t,dp(3,:));
% plot(t,dp_des(1,:)),
% plot(t,dp_des(2,:));
% plot(t,dp_des(3,:));
% legend("$\dot{p}_x$","$\dot{p}_y$","$\dot{p}_z$",...
%         "$\dot{p}_{x,des}$","$\dot{p}_{y,des}$","$\dot{p}_{z,des}$","Interpreter","Latex");

subplot(5,1,1)
hold on, grid on, ylabel("$p \ [m]$","Interpreter","Latex");
plot(t,p(1,:)),
plot(t,p(2,:));
plot(t,p(3,:));
xline(t_salient,'--k');
legend("$p_x$","$p_y$","$p_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");

subplot(5,1,2)
hold on, grid on, ylabel("$\dot{p} \ [m/s]$","Interpreter","Latex");
plot(t,dp(1,:)), hold on;
plot(t,dp(2,:));
plot(t,dp(3,:));
xline(t_salient,'--k');
legend("$\dot{p}_x$","$\dot{p}_y$","$\dot{p}_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");

subplot(5,1,3)
hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex");
plot(t,f_ext(1,:)), hold on;
plot(t,f_ext(2,:));
plot(t,f_ext(3,:));
xline(t_salient,'--k');
legend("$f_x$","$f_y$","$f_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");

subplot(5,1,4)
plot(t,r)
xline(t_salient,'--k');
yline([residual_threshold -residual_threshold],'--r')
grid on, ylabel("$r \ [W]$","Interpreter","Latex");

subplot(5,1,5)
plot(t,P_ext)
xline(t_salient,'--k');
grid on, ylabel("$P_{ext} \ [W]$","Interpreter","Latex"), xlabel("$t \ [s]$","Interpreter","Latex");

end