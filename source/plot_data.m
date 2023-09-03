function plot_data(p,dp,p_des,dp_des,f_ext,u_ext,sigma,r,residual_threshold,P_ext,t_salient,t,save_video)

time_axis = sort([t_salient, 0, 10]);
if save_video
    v = VideoWriter("videos/robot_plots.avi");
    v.FrameRate=floor(size(t,2)/max(t));
    v.Quality=100;
    open(v)
end

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.WindowState='max';
set(0,'DefaultLineLineWidth',1)

subplot(3,1,1)
hold on, grid on, ylabel("$p^{ee} \ [m]$","Interpreter","Latex");
plot(t,p(1,:)),
plot(t,p(2,:));
plot(t,p(3,:));
xline(t_salient,'--k');
legend("$p^{ee}_x$","$p^{ee}_y$","$p^{ee}_z$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(3,1,2)
hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex");
plot(t,f_ext(1,:)), hold on;
plot(t,f_ext(2,:));
plot(t,f_ext(3,:));
xline(t_salient,'--k');
legend("$f_{ext,x}$","$f_{ext,y}$","$f_{ext,z}$","Interpreter","Latex","Orientation","Horizontal", ...
    "Location","south");
xticks(time_axis)
xlim([0 max(t)])

subplot(3,1,3)
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
grid on, ylabel("$\sigma \ [W]$","Interpreter","Latex"), xlabel("$t \ [s]$","Interpreter","Latex");
xticks(time_axis)
xlim([0 max(t)])


if save_video
    f=figure;
    f.WindowState='max';
    set(0,'DefaultLineLineWidth',1)
    
    subplot(3,1,1), xlim([0 max(t)]), ylim([min(p,[],'all') max(p,[],'all')])
    hold on, grid on, ylabel("$p^{ee} \ [m]$","Interpreter","Latex");
    xticks(time_axis)
    xline(t_salient,'--k');

    h1=animatedline('Color',[0 0.4470 0.7410]);
    h2=animatedline('Color',[0.8500 0.3250 0.0980]);
    h3=animatedline('Color',[0.9290 0.6940 0.1250]);

    subplot(3,1,2), xlim([0 max(t)]), ylim([min(f_ext,[],'all') max(f_ext,[],'all')])
    hold on, grid on, ylabel("$f_{ext} \ [N]$","Interpreter","Latex");
    xticks(time_axis)
    xline(t_salient,'--k');

    h4=animatedline('Color',[0 0.4470 0.7410]);
    h5=animatedline('Color',[0.8500 0.3250 0.0980]);
    h6=animatedline('Color',[0.9290 0.6940 0.1250]);

    subplot(3,1,3), xlim([0 max(t)]), ylim([min(sigma) max(sigma)]), 
    hold on, grid on, ylabel("$\sigma \ [W]$","Interpreter","Latex"), xlabel("$t \ [s]$","Interpreter","Latex");
    xline(t_salient,'--k');
    xticks(time_axis);
    yline([residual_threshold -residual_threshold],'--r')
    
    h7=animatedline('Color',[0 0.4470 0.7410]);

    for k=1:size(p,2)
        addpoints(h1,t(k),p(1,k));
        addpoints(h2,t(k),p(2,k));
        addpoints(h3,t(k),p(3,k));
        addpoints(h4,t(k),f_ext(1,k));
        addpoints(h5,t(k),f_ext(2,k));
        addpoints(h6,t(k),f_ext(3,k));
        addpoints(h7,t(k),sigma(k));
        drawnow;
        if k < size(p,2)
            if abs(sigma(k+1))>residual_threshold && abs(sigma(k))<residual_threshold
                if norm(f_ext(:,k+1))>0
                    plot(t(k+1),sigma(k+1),'*g');
                else
                    plot(t(k+1),sigma(k+1),'*k');
                end
            end
        end
        frame = getframe(gcf);
        writeVideo(v,frame);
    end

end
end