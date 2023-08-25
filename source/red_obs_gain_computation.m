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

%% Find bounds of M and C for computation of reduced-order oberver gain
% compute symbolically M and C
syms q1 q2 q3 dq1 dq2 dq3 real
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
[~,A] = DHMatrix_sym(DHTABLE+[zeros(3,3) q]);
M = simplify([modNE_sym([0;0;0],[0;0;0],[1;0;0],m,I,A,pc,0) modNE_sym([0;0;0],[0;0;0],[0;1;0],m,I,A,pc,0) modNE_sym([0;0;0],[0;0;0],[0;0;1],m,I,A,pc,0)]);
C = simplify([modNE_sym(dq,[1;0;0],[0;0;0],m,I,A,pc,0) modNE_sym(dq,[0;1;0],[0;0;0],m,I,A,pc,0) modNE_sym(dq,[0;0;1],[0;0;0],m,I,A,pc,0)]);
matlabFunction(M,'File','get_inertia_matrix.m','Vars',[q2 q3]);
matlabFunction(C,'File','get_c_factorization_matrix.m','Vars',[q2 q3 dq1 dq2 dq3]);
% set an arbitrary maximum joint velocity and an arbitrary eta
v_max = 2;  % [rad/s]
eta = 1;  % [rad/s]
%% find c0bar
q2_l=-pi; q2_u=pi;
q3_l=-pi; q3_u=pi;
dq_l=ones(3,1)*-v_max; dq_u=ones(3,1)*v_max;
%x is [q2,q3,dq1,dq2,dq3]
x0=[pi/4,pi/4,1,1,1]';
% tried with: active-set, sqp, interior point and always the same result.
% tried with rand(5,1) as x0 and still the same result.
sol=fmincon(@(x) -norm(get_c_factorization_matrix(x(1),x(2),x(3),x(4),x(5)))/norm(x(3:5)),...
    x0,[], [], [], [], [q2_l;q3_l;dq_l], [q2_u;q3_u;dq_u]);
c0bar = norm(get_c_factorization_matrix(sol(1),sol(2),sol(3),sol(4),sol(5)))/norm(sol(3:5))
%% study eigenvalues of M
lambda=eig(M);
q2span = 0:0.01:2*pi;
q3span = 0:0.01:2*pi;
figure
plot(q3span,subs(lambda(1),q3,q3span)),hold on,
plot(q3span,subs(lambda(2),q3,q3span)),legend("first","second")
min_first=min(subs(lambda(1),q3,q3span));
max_second=max(subs(lambda(2),q3,q3span));

figure
f = @(x,y) cos(2*x + y)/2 + cos(y)/2 + (61*cos(x + y)^2)/240 + (247*cos(x)^2)/120 + 3/8;
fs=fsurf(f,[0 2*pi 0 2*pi]);
min_third=min(fs.ZData);
max_third=max(fs.ZData);
lambda_1 = min(min_first,min_third)
lambda_2 = max(max_second,max_third)
%% compute gain and region of exp stability
% region of exp stability
epsilon = eta*sqrt(lambda_1/lambda_2);
K0 = c0bar*(v_max+eta)/(2*lambda_1);