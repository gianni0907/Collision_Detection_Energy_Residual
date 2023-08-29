clear variables
clc
% script to generate functions to retrieve the position of the end-effector,
% the linear part of the end effector jacobian and its derivative
syms q1 q2 q3 dq1 dq2 dq3 real
q = [q1 q2 q3]';
dq = [dq1 dq2 dq3]';
l = [0.5 0.5 0.4]';
DHTable = [pi/2 0       l(1)    q1;
           0    l(2)    0       q2;
           0    l(3)    0       q3];
[T,A] = DHMatrix(DHTable);
pee = T(1:3,4);
matlabFunction(pee,'File','get_pee.m','Vars',[q1 q2 q3]);
Jee = simplify(jacobian(pee,q));
matlabFunction(Jee,'File','get_Jee.m','Vars',[q1 q2 q3]);
dJee = simplify(diff(Jee,q1)*dq1 + diff(Jee,q2)*dq2 + diff(Jee,q3)*dq3);
matlabFunction(dJee,'File','get_dJee.m','Vars',[q1 q2 q3 dq1 dq2 dq3]);