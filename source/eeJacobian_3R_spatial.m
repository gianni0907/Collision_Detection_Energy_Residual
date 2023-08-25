clear variables
clc

syms q1 q2 q3 dq1 dq2 dq3 l1 l2 l3 real
q = [q1 q2 q3]';
dq = [dq1 dq2 dq3]';
l = [0.5 0.5 0.4]';
DHTable = [pi/2 0       l1    q1;
           0    l2    0       q2;
           0    l3    0       q3];
[T,A] = DHMatrix_sym(DHTable);
pee = T(1:3,4);
Jee = simplify(jacobian(pee,q));
matlabFunction(Jee,'File','get_Jee.m','Vars',[q1 q2 q3]);
dJee = simplify(diff(Jee,q1)*dq1 + diff(Jee,q2)*dq2 + diff(Jee,q3)*dq3);
matlabFunction(dJee,'File','get_dJee.m','Vars',[q1 q2 q3 dq1 dq2 dq3]);