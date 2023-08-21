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
% iterate over all the possible joint configurations for q2 and q3
% and over all possible joint velocities
lambda_1 = 1e6;
lambda_2 = -1e6;
c0bar = 0;
for q2i=0.1:0.1:2*pi
    for q3i=0.1:0.1:2*pi
        eig_M = sort(eig(get_inertia_matrix(q2i,q3i)));
        min_eig = eig_M(1);
        max_eig = eig_M(end);
        if min_eig/2 < lambda_1
            lambda_1 = min_eig/2;
        end
        if max_eig/2 > lambda_2
            lambda_2 = max_eig/2;
        end
        for dq1i=-v_max:0.1:v_max
            for dq2i=-v_max:0.1:v_max
                for dq3i=-v_max:0.1:v_max
                    c0 = norm(get_c_factorization_matrix(q2i,q3i,dq1i,dq2i,dq3i))/norm([dq1i dq2i dq3i]);
                    if c0 > c0bar
                        c0bar = c0;
                    end
                end
            end
        end
    end
end
% region of attraction
epsilon = eta*sqrt(lambda_1/lambda_2);
K0 = c0bar*(v_max+eta)/(2*lambda_1);