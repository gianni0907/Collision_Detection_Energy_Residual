clear variables
close all
clc

% Numerical definition of dynamic parameters
g0=9.81; % acceleration of gravity [m/s^2]
N=3;  % number of joints
l = [0.5 0.5 0.4]'; % link lengths [m]
dc = [l(1)/2 l(2)/2 l(3)/2]'; % link CoMs (on local x axis) [m]
m = [15 10 5]'; % link masses [kg]
r = [0.2 0.1 0.1]'; % radius [m] of cylinder links with uniform mass 
I = zeros(3,3,N); % tensor containing the Inertia matrix of the links
I(3,3,1) = (1/2)*m(1)*r(1)^2; % [kg*m^2]
I(1,1,2) = (1/2)*m(2)*r(2)^2;
I(2,2,2) = (1/12)*m(2)*(3*r(2)^2+l(2)^2);
I(3,3,2) = I(2,2,2);
I(1,1,3) = (1/2)*m(3)*r(3)^2;
I(2,2,3) = (1/12)*m(3)*(3*r(3)^2+l(3)^2);
I(3,3,3) = I(2,2,3);
pc = [0             -l(2)+dc(2) -l(3)+dc(3);
      -l(1)+dc(1)   0           0;
      0             0           0];

q = [0 0 0]';
dq = [0 0 0]';
ddq = [0 0 0]';

DHTABLE = [ pi/2      0         l(1)     q(1);
             0        l(2)        0      q(2);
             0        l(3)        0      q(3)];

[T,A] = DHMatrix(DHTABLE);
R = A(1:3,1:3,:);
p = [R(:,:,1)'*A(1:3,4,1) R(:,:,2)'*A(1:3,4,2) R(:,:,3)'*A(1:3,4,3)];
I(:,:,1) = R(:,:,1)'*I(:,:,1);

[M,C,g]=get_dyn_terms(q(2),q(3),dq(1),dq(2),dq(3));
u=modNE(dq,dq,ddq,m,I,R,p,pc,g0);