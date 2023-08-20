clear variables
close all
clc
%% Numerical values

% Controller Parameters
Kp = 100; % PD gains
Kd = 50;

% Kinetic parameters
L1=0.5; % link lengths [m]  
L2=0.5;
L3=0.4;

dc1=L1/2; % link CoMs [m]
dc2=L2/2; 
dc3=L3/2; 

%% parameters for the liner trajectory
p_i=[0.8, 0., 0.5];
check_init = (p_i(1)^2+p_i(2)^2+(p_i(3)-L1)^2-L2^2-L3^2)/(2*L2*L3);
if abs(check_init)>1
    disp('**********initial point out of ws********')
    return
end
p_f=[0.,0.8,0.5];
check_fin = (p_f(1)^2+p_f(2)^2+(p_f(3)-L1)^2-L2^2-L3^2)/(2*L2*L3);
if abs(check_fin)>1
    disp('**********final point out of ws********')
    return
end
Timeinterval=[7,9];
%% initial conditions
q0 = inv_kin(p_i);
dq0 = [0., 0., 0.]';
x0=[q0;dq0];
ddq0 = [0,0,0]';
%% parameters for the polynomial trajectory
Waypoints = [q0(1) pi/2 pi/4 0 -pi/2; q0(2) pi/4 pi/2 3*pi/4 pi/2; q0(3) pi/2 pi pi/2 -pi/2];
Velocities = [dq0(1) 2 1 0 -2;dq0(2) 0 2 0 -2;dq0(3) 2 2 0 0];
Timepoints = [0 3 5 7 10];

Ko=10; % time constant for the residual

%% Observer parameters
finite_diff = -1; % if this variable is >0, finite differences is used. Otherwise, observer
full_state=2;
observer=3;
lambda_1 = 0.094789482226257512094598012320061 / 2;
lambda_2 = 3.6875 / 2;
c0_bar = 1.962390280742725;
vmax=2;
vbar=2;
r0=1;
eta=1;
epsilon=eta*sqrt(lambda_1/lambda_2);
K0=c0_bar*(vmax+eta)/(2*lambda_1);
x2hat0=zeros(3,1);
z0=x2hat0-K0*q0;
%% robot dynamics parameters and structures for NE algorithm

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

m=[m1 m2 m3]';
rc = zeros(3,1,3);
rc(:,:,1)=[0, -L1+dc1, 0]';
rc(:,:,2)=[-L2+dc2, 0, 0]';
rc(:,:,3)=[-L3+dc3, 0, 0]';

I = zeros(3,3,3);
I(:,:,1)=diag([0 I1zz 0]);
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
%% create robot model for kinematics
dhparams = [0   	pi/2	L1   	0;
            L2      0       0       0;
            L3      0       0   	0];

robot = rigidBodyTree;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;

addBody(robot,body2,'body1');
addBody(robot,body3,'body2');
%% run and plot simulation data

% run simulation
out=sim('simulation',[0 10]);

f=figure;
f.Position=[276.2,77,988.8,658.4];
subplot(321), 
plot(out.tout,squeeze(out.state(1,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.qdesired(1,1,:)))
legend("$q_1$","$q_{1,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad]")
title("Desired vs actual joint position")

subplot(323), 
plot(out.tout,squeeze(out.state(2,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.qdesired(2,1,:)))
legend("$q_2$","$q_{2,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad]")

subplot(325), 
plot(out.tout,squeeze(out.state(3,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.qdesired(3,1,:)))
legend("$q_3$","$q_{3,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad]")

subplot(322), 
plot(out.tout,squeeze(out.state(4,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.dqdesired(1,1,:)))
legend("$\dot{q}_1$","$\dot{q}_{1,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")
title("Desired vs actual joint velocity")

subplot(324), 
plot(out.tout,squeeze(out.state(5,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.dqdesired(2,1,:)))
legend("$\dot{q}_2$","$\dot{q}_{2,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")

subplot(326), 
plot(out.tout,squeeze(out.state(6,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.dqdesired(3,1,:)))
legend("$\dot{q}_3$","$\dot{q}_{3,d}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.Position=[276.2,77,988.8,658.4];
subplot(311), 
plot(out.tout,squeeze(out.qdesired(1,1,:)-out.state(1,1,:)),'--'), grid on,
legend("$e_1$","Interpreter","Latex")
xlabel("[s]"), ylabel("[rad]")
title("Error on joint position")

subplot(312), 
plot(out.tout,squeeze(out.qdesired(2,1,:)-out.state(2,1,:)),'--'), grid on,
legend("$e_2$","Interpreter","Latex")
xlabel("[s]"), ylabel("[rad]")

subplot(313), 
plot(out.tout,squeeze(out.qdesired(3,1,:)-out.state(3,1,:)),'--'), grid on,
legend("$e_3$","Interpreter","Latex")
xlabel("[s]"), ylabel("[rad]")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.Position=[276.2,77,988.8,658.4];
subplot(311), 
plot(out.tout,squeeze(out.state(4,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.dq_est(1,1,:)))
legend("$\dot{q}_1$","$\dot{q}_{1,est}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")
if finite_diff >0
    title("actual vs estimated joint velocity using finite differences")
else
    title("actual vs estimated joint velocity using reduced observer")
end

subplot(312), 
plot(out.tout,squeeze(out.state(5,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.dq_est(2,1,:)))
legend("$\dot{q}_2$","$\dot{q}_{2,est}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")

subplot(313), 
plot(out.tout,squeeze(out.state(6,1,:)),'--'), hold on, grid on
plot(out.tout,squeeze(out.dq_est(3,1,:)))
legend("$\dot{q}_3$","$\dot{q}_{3,est}$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

f=figure;
f.Position=[276.2,77,988.8,658.4];
subplot(311), 
plot(out.tout,squeeze(out.state(4,1,:)-out.dq_est(1,1,:))),grid on
legend("$e_1$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")
title("error=actual-estimated")

subplot(312), 
plot(out.tout,squeeze(out.state(5,1,:)-out.dq_est(2,1,:))),grid on
legend("$e_2$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")

subplot(313), 
plot(out.tout,squeeze(out.state(6,1,:)-out.dq_est(3,1,:))),grid on
legend("$e_3$","Interpreter","latex")
xlabel("[s]"), ylabel("[rad/s]")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
subplot(311)
plot(out.tout,squeeze(out.input(1,1,:))), hold on, grid on
plot(out.tout,squeeze(out.input(2,1,:)))
plot(out.tout,squeeze(out.input(3,1,:)))
legend("$\tau_1$","$\tau_2$","$\tau_3$","Interpreter","latex")
xlabel("[s]"), ylabel("[Nm]")
title("Input torques")
subplot(312)
plot(out.tout,squeeze(out.f_ext(1,1,:))), hold on, grid on
plot(out.tout,squeeze(out.f_ext(2,1,:)))
plot(out.tout,squeeze(out.f_ext(3,1,:)))
legend("$f_{ext,x}$","$f_{ext,y}$","$f_{ext,z}$","Interpreter","latex")
xlabel("[s]"), ylabel("[N]")
title("External force")
subplot(313)
plot(out.tout,squeeze(out.tau_ext(1,1,:))), hold on, grid on
plot(out.tout,squeeze(out.tau_ext(2,1,:)))
plot(out.tout,squeeze(out.tau_ext(3,1,:)))
legend("$\tau_{ext,1}$","$\tau_{ext,2}$","$\tau_{ext,3}$","Interpreter","latex")
xlabel("[s]"), ylabel("[N]")
title("Resulting joint torque")

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
subplot(211)
plot(out.tout,out.residual), grid on
title("Residual value"), xlabel("[s]"), ylabel("[W]")
subplot(212)
plot(out.tout,out.p_ext), grid on
title("External power"), xlabel("[s]"), ylabel("[W]")

%% show the robot
cnt = size(out.tout,1);
configs=zeros(cnt,3);
for i=1:cnt
    configs(i,:)=out.state(1:3,1,i);
end
figure
conf=homeConfiguration(robot);
show(robot,conf);
hold on
cla
plot3(p_i(1),p_i(2),p_i(3),'.','MarkerSize',18)
plot3(p_f(1),p_f(2),p_f(3),'.','MarkerSize',18)
plot3([p_i(1),p_f(1)],[p_i(2),p_f(2)],[p_i(3),p_f(3)],'LineWidth',1.2)
show_robot(configs,out.tout);
