clear variables
close all
clc

%% Numerical definition of dynamic parameters
g0=9.81; % acceleration of gravity [m/s^2]
N=3;  % number of joints
l = [0.5 0.5 0.4]'; % link lengths [m]
dc = [l(1)/2 l(2)/2 l(3)/2]'; % link CoMs (on local x axis) [m]
m = [15 10 5]'; % link masses [kg]
radius = [0.2 0.1 0.1]'; % radius [m] of cylinder links with uniform mass 
I = zeros(3,3,N); % tensor containing the Inertia matrix of the links
I(2,2,1) = (1/2)*m(1)*radius(1)^2; % [kg*m^2]
I(1,1,2) = (1/2)*m(2)*radius(2)^2;
I(2,2,2) = (1/12)*m(2)*(3*radius(2)^2+l(2)^2);
I(3,3,2) = I(2,2,2);
I(1,1,3) = (1/2)*m(3)*radius(3)^2;
I(2,2,3) = (1/12)*m(3)*(3*radius(3)^2+l(3)^2);
I(3,3,3) = I(2,2,3);
pc = [0             -l(2)+dc(2) -l(3)+dc(3);
      -l(1)+dc(1)   0           0;
      0             0           0];

DHTABLE = [ pi/2      0         l(1)     0;
             0        l(2)        0      0;
             0        l(3)        0      0];

%% create the robot model
dhparams = [DHTABLE(:,2) DHTABLE(:,1) DHTABLE(:,3:4)];
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
mat1 = [elem_rot_mat('x',pi/2) [0 -l(1)/2 0]';
        zeros(1,3)              1];
mat2 = [elem_rot_mat('y',pi/2)  [-l(2)/2 0 0]';
        zeros(1,3)               1];
mat3 = [elem_rot_mat('y',pi/2)  [-l(3)/2 0 0]';
        zeros(1,3)               1];
cylinder1 = collisionCylinder(radius(1),l(1));
cylinder2 = collisionCylinder(radius(2),l(2));
cylinder3 = collisionCylinder(radius(3),l(3));
cylinder1.Pose = mat1;
cylinder2.Pose = mat2;
cylinder3.Pose = mat3;
% addCollision(robot.Bodies{1},cylinder1);
% addCollision(robot.Bodies{2},cylinder2);
% addCollision(robot.Bodies{3},cylinder3);

%% set parameters value for the simulation
% implement switching logic for reduced-order observer gain scheduling
c0bar = 1.962390280720058;
lambda_1 = 0.094789482226257512094598012320061;
lambda_2 = 59/16; %3.6875
epsilon = 0.160981780095266;
vmax = 2;  % [rad/s] useless in case of switching logic
eta = 1;  % [rad/s]
vbar = 2*eta+1;  % [rad/s]
% set gain values for pd feedback, residual and reduced-order observer
Kp = 1e2;
Kd = 5e1;
Ko = 5e1;
K0 = c0bar*(vmax+eta)/(2*lambda_1);

% choose method for velocity estimation:
% 0: full state implmentation, velocity is available
% 1: use reduced order observer for estimation
% 2: use finite differences for estimation
estimate_velocity=2;

% threshold for the residual to detect a collision
residual_threshold = 3;
%% Define Cartesian motion
% first motion: a circle in the y-z plane
c = [0.4; 0.2; 0.7];  % center of the circle
r_c = 0.3;  % circle radius
T = 2; % period
t0 = 2; % start of motion time
p_init=c;
p0 = [c(1); c(2)+r_c; c(3)];
% rest phase: 
t1 = t0+T;
T_stop1 = 3;
t2 = t1+T_stop1;
% second motion:
p1 = [0.8; c(2); c(3)];
T_line1 = 1;
% rest phase:
t3 = t2+T_line1;
T_stop2 = 3;
t4 = t3+T_stop2;
% third motion: 
p2 = [c(1);c(2)-r_c;c(3)];
T_line2 = 1.5;
t5 = t4+T_line2;

% check whether the points are out of workspace
if ~(check_in_workspace(p0,l) && check_in_workspace(p1,l) && ...
        check_in_workspace(p2,l) && ...
        check_in_workspace([c(1);c(2);c(3)+r_c],l) && ...
        check_in_workspace([c(1);c(2);c(3)-r_c],l))
    return;
end

% initialization of the state integrator
soln_type = 'pn';
q0 = inverse_kinematics(p_init,l,soln_type);
dq0 = [0.5 -0.5 1]';
x0 = [q0; dq0];
% initialization of the reduced observer integrator
z0 = -K0*q0;
% initialization of the residual intergrator
[~,A0] = DHMatrix(DHTABLE+[zeros(3,3) q0]);
M0 = [modNE([0;0;0],[0;0;0],[1;0;0],m,I,A0,pc,0) modNE([0;0;0],[0;0;0],[0;1;0],m,I,A0,pc,0) modNE([0;0;0],[0;0;0],[0;0;1],m,I,A0,pc,0)];
T0 = 0.5*dq0'*M0*dq0;

% initialization of the discrete integrator for the switching logic
s0 = ceil(norm(dq0)/vbar);

%% run the simulation and show the results
out=sim('simulation.slx',[0 15]);

t = out.tout';
f_ext = out.f_ext';
q_des = reshape(out.x_des(1:3,:,:),N,size(out.x_des,3));
dq_des = reshape(out.x_des(4:6,:,:),N,size(out.x_des,3));
p_des = reshape(out.p_des(1:3,:,:),N,size(out.x_des,3));
dp_des = reshape(out.dp_des,N,size(out.x_des,3));
p = reshape(out.p,N,size(out.x_des,3));
dp = reshape(out.dp,N,size(out.x_des,3));

P_ext = diag(dp'*f_ext)';

q = reshape(out.x(1:3,:,:),N,size(out.x,3));
dq = reshape(out.x(4:6,:,:),N,size(out.x,3));
u = reshape(out.u,N,size(out.u,3));
r = out.r';

x2hat = out.x2hat';

t_salient=[t0 t1 t2 t3 t4 t5];
plot_data(p,dp,p_des,dp_des,f_ext,r,residual_threshold,P_ext,t_salient,t);

plot_joint_level(q_des,dq_des,q,dq,x2hat,u,t);

% figure
% show(robot,homeConfiguration(robot));
% cla
% hold on
% plot3(p0(1),p0(2),p0(3),'.','MarkerSize',18);
% t_c=0:0.05:T;
% plot3(c(1)*ones(size(t_c,2),1),c(2)+r_c*cos(2*pi/T*t_c),c(3)+r_c*sin(2*pi/T*t_c),'LineWidth',1.2)
% plot3(p1(1),p1(2),p1(3),'.','MarkerSize',18);
% plot3(p2(1),p2(2),p2(3),'.','MarkerSize',18);
% plot3([p0(1),p1(1)],[p0(2),p1(2)],[p0(3),p1(3)],'LineWidth',1.2);
% plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'LineWidth',1.2);
% 
% robot_motion(robot,q,t);