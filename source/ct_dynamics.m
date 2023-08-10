function dx = ct_dynamics(x,u,m,I,A,pc,g0)
% return the continuous time dynamics given state and input
e0 = zeros(3,1);
e1 = [1 0 0]';
e2 = [0 1 0]';
e3 = [0 0 1]';
x2=x(4:6);
%get inertia matrix M from modNE algorithm
M = [modNE(e0,e0,e1,m,I,A,pc,0) modNE(e0,e0,e2,m,I,A,pc,0) modNE(e0,e0,e3,m,I,A,pc,0)];
c = modNE(x2,x2,e0,m,I,A,pc,0);
g = modNE(e0,e0,e0,m,I,A,pc,g0);

dx=[x2;
    inv(M)*(-c-g+u)];
end