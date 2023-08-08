function dx = ct_dynamics(x,u)
% return the continuous time dynamics given state  and input

[M,C,g]=get_dyn_terms(x(2),x(3),x(4),x(5),x(6));
x1=x(1:3);
x2=x(4:6);
dx=[x2;
    inv(M)*(-C*x2-g+u)];
end