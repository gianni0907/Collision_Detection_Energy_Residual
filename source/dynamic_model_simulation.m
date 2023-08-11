clear variables
close all
clc

%% Numerical definition of dynamic parameters
g0=9.81; % acceleration of gravity [m/s^2]
N=3;  % number of joints
l = [0.5 0.5 0.4]'; % link lengths [m]
dc = [l(1)/2 l(2)/2 l(3)/2]'; % link CoMs (on local x axis) [m]
m = [15 10 5]'; % link masses [kg]
r = [0.2 0.1 0.1]'; % radius [m] of cylinder links with uniform mass 
I = zeros(3,3,N); % tensor containing the Inertia matrix of the links
I(2,2,1) = (1/2)*m(1)*r(1)^2; % [kg*m^2]
I(1,1,2) = (1/2)*m(2)*r(2)^2;
I(2,2,2) = (1/12)*m(2)*(3*r(2)^2+l(2)^2);
I(3,3,2) = I(2,2,2);
I(1,1,3) = (1/2)*m(3)*r(3)^2;
I(2,2,3) = (1/12)*m(3)*(3*r(3)^2+l(3)^2);
I(3,3,3) = I(2,2,3);
pc = [0             -l(2)+dc(2) -l(3)+dc(3);
      -l(1)+dc(1)   0           0;
      0             0           0];

DHTABLE = [ pi/2      0         l(1)     0;
             0        l(2)        0      0;
             0        l(3)        0      0];

% initialization of the state vector
% q0 = rand(3,1)*2*pi-pi;
% dq0 = rand(3,1)*10-5;
q0 = [0 0 0]';
dq0 = [0 0 0]';
x0 = [q0' dq0']';
DHTABLE = DHTABLE+[zeros(3,3) q0]
% define joint space waypoints of the trajectory, associated instant of
% time and velocity boundary conditions
wp = [q0(1), pi/2, pi;
      q0(2), pi/4, pi/2;
      q0(3), pi/2, 0];
tp = [0 5 10];
boundary_vel = zeros(3,3);

% set gain values for pd feedback
Kp = 100;
Kd = 50;

%% run the simulation and show the results 
out=sim('simulation.slx');
q_des = reshape(out.x_des(1:3,:,:),N,size(out.x_des,3))';
dq_des = reshape(out.x_des(4:6,:,:),N,size(out.x_des,3))';
q_act = reshape(out.x(1:3,:,:),N,size(out.x,3))';
dq_act = reshape(out.x(4:6,:,:),N,size(out.x,3))';
u = reshape(out.u,N,size(out.u,3))';
plot_data(q_des,dq_des,q_act,dq_act,u,out.tout);
robot_motion(DHTABLE,l,r,q_act);

