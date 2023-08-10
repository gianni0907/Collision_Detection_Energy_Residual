clear variables
close all
clc
%% Numerical values

Kp = 50; % PD gains
Kd = 10; 

Ko=0.1; % time constant for the residual

L1=0.5; % link lengths [m]  
L2=0.5;
L3=0.4;

dc1=L1/2; % link CoMs [m]
dc2=L2/2; 
dc3=L3/2; 

m1=15; % link masses [kg]
m2=10;
m3=5;

r1=0.2;  % links as full cylinders with uniform mass of radius r [m] 
I1zz=(1/2)*m1*r1^2; % [kg*m^2]

r2=0.1;
I2xx=(1/2)*m2*r2^2;
I2yy=(1/12)*m2*(3*r2^2+L2^2);
I2zz=I2yy;

r3=0.1;
I3xx=(1/2)*m3*r3^2;
I3yy=(1/12)*m3*(3*r3^2+L3^2);
I3zz=I3yy;

g0=9.81; % acceleration of gravity [m/s^2]

DHTABLE = [ pi/2   0     L1     0;
             0     L2    0      0;
             0     L3    0      0];
%% build structures for NE
q0 = [0,pi/4,pi/4]';
dq0 = [0,1,1]';
ddq0 = [0,0,0]';
m=[m1 m2 m3]';
rc = zeros(3,1,3);
rc(:,:,1)=[0, -L1+dc1, 0]';
rc(:,:,2)=[-L2+dc2, 0, 0]';
rc(:,:,3)=[-L3+dc3, 0, 0]';

I = zeros(3,3,3);
I(:,:,1)=diag([0 0 I1zz]);
I(:,:,2)=diag([I2xx I2yy I2zz]);
I(:,:,3)=diag([I3xx I3yy I3zz]);

DHTABLE(:,end)=q0;

% compute initial kinetic energy to initialize the residual
Ai = DHMatrix(DHTABLE);
M = zeros(3,3);
M(:,1) = newton_euler(zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1), ...
        Ai,zeros(3,1),[1;0;0],m,I,rc);
M(:,2) = newton_euler(zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1), ...
        Ai,zeros(3,1),[0;1;0],m,I,rc);
M(:,3) = newton_euler(zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1), ...
        Ai,zeros(3,1),[0;0;1],m,I,rc);
K_init = 0.5*dq0'*M*dq0;
% Ai=DHMatrix(DHTABLE);
% S=zeros(3,3);
% S(:,1)=0.5*(newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,dq0+[1;0;0],ddq0,m,I,rc)- ...
%             newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,dq0,ddq0,m,I,rc)- ...
%             newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,[1;0;0],ddq0,m,I,rc));
% S(:,2)=0.5*(newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,dq0+[0;1;0],ddq0,m,I,rc)- ...
%             newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,dq0,ddq0,m,I,rc)- ...
%             newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,[0;1;0],ddq0,m,I,rc));
% S(:,3)=0.5*(newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,dq0+[0;0;1],ddq0,m,I,rc)- ...
%             newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,dq0,ddq0,m,I,rc)- ...
%             newton_euler([0;0;0],[0;0;0],[0;0;0],[0;0;0],[0;0;0],Ai,[0;0;1],ddq0,m,I,rc));
% [~,C,~] = get_dyn_terms(q0(2),q0(3),dq0(1),dq0(2),dq0(3));
out=sim('simulation',[0 10]);
%% plot the simulation data
figure
subplot(211), 
plot(out.tout,squeeze(out.state(1,1,:)),'r--'), hold on, grid on
plot(out.tout,squeeze(out.state(2,1,:)),'g--')
plot(out.tout,squeeze(out.state(3,1,:)),'b--')
plot(out.tout,squeeze(out.qdesired(1,1,:)),'r')
plot(out.tout,squeeze(out.qdesired(2,1,:)),'g')
plot(out.tout,squeeze(out.qdesired(3,1,:)),'b')

legend("$q_1$","$q_2$","$q_3$","$q_{1,d}$","$q_{2,d}$","$q_{3,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad]")
title("Joint position")

subplot(212), 
plot(out.tout,squeeze(out.state(4,1,:)),'r--'), hold on, grid on
plot(out.tout,squeeze(out.state(5,1,:)),'g--')
plot(out.tout,squeeze(out.state(6,1,:)),'b--')
plot(out.tout,squeeze(out.dqdesired(1,1,:)),'r')
plot(out.tout,squeeze(out.dqdesired(2,1,:)),'g')
plot(out.tout,squeeze(out.dqdesired(3,1,:)),'b')

legend("$\dot{q}_1$","$\dot{q}_2$","$\dot{q}_3$", ...
    "$\dot{q}_{1,d}$","$\dot{q}_{2,d}$","$\dot{q}_{3,d}$","Interpreter","latex")

xlabel("[s]"), ylabel("[rad/s]")
title("Joint velocity")

figure
subplot(211)
plot(out.tout,squeeze(out.input(1,1,:))), hold on, grid on
plot(out.tout,squeeze(out.input(2,1,:)))
plot(out.tout,squeeze(out.input(3,1,:)))
legend("$\tau_1$","$\tau_2$","$\tau_3$","Interpreter","latex")
xlabel("[s]"), ylabel("[Nm]")
title("Input torques")
subplot(212)
plot(out.tout,out.residual), grid on
title("Residual value"), xlabel("[s]"), ylabel("[W]")

%% show the robot
cnt = size(out.tout,1);
configs=zeros(cnt,3);
for i=1:cnt
    configs(i,:)=out.state(1:3,1,i);
end
show_robot(configs,out.tout);