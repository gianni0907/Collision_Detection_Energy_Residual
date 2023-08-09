function u=newton_euler(w_0,dw_0,a_0,f_n1,tau_n1,Ai,dq,ddq,m,I,rc)
% NE algorithm for robot dynamics with revolute joints
% inputs:
% - w_0, dw_0: angular velocity and acceleration of the base
%  - a_0: acceleration of the base (set this to -g to consider gravity)
%  - f_n1,tau_n1: force and torque vectors applied at the end effector, 0 if not in contact with environment
%  - Ai: 4x4xn matrix with the 4x4 DH homogeneous matrices
%  - dq, ddq: joint velocity and acceleration
%  - m: n-dimensional array with link masses
%  - I: 3x3xn matrix containing inertia matrices
%  - rc: 3x1xn matrix with CoM vectors of link i expressed in frame i

n=size(Ai,3);

R=zeros(3,3,n);
for i=1:n
    R(:,:,i)=Ai(1:3,1:3,i);
end

w=zeros(3,1,n); 
w(:,:,1)=R(:,:,1)'*(w_0+[0;0;dq(1)]);

dw=zeros(3,1,n); 
dw(:,:,1)=R(:,:,1)'*(dw_0+[0;0;ddq(1)] + dq(1)*cross(w_0,[0;0;1]));

a=zeros(3,1,n);
a(:,:,1)=R(:,:,1)'*a_0 + cross(dw(:,:,1),R(:,:,1)'*Ai(1:3,4,1)) + cross(w(:,:,1),cross(w(:,:,1),R(:,:,1)'*Ai(1:3,4,1)));

ac=zeros(3,1,n);
ac(:,:,1)= a(:,:,1) + cross(dw(:,:,1),rc(:,:,1)) + cross(w(:,:,1),cross(w(:,:,1),rc(:,:,1)));

%forward recursion
for i=2:n
    w(:,:,i)=R(:,:,i)'*(w(:,:,i-1)+[0;0;dq(i)]);
    dw(:,:,i)=R(:,:,i)'*( dw(:,:,i-1)+[0;0;ddq(i)] + dq(i)*(Smtrx(w(:,:,i-1))*[0;0;1]));
    a(:,:,i)=R(:,:,i)'*a(:,:,i-1) + cross(dw(:,:,i),R(:,:,i)'*Ai(1:3,4,i)) + cross(w(:,:,i),cross(w(:,:,i), R(:,:,i)'*Ai(1:3,4,i)));
    ac(:,:,i)= a(:,:,i) + cross(dw(:,:,i),rc(:,:,i)) + cross(w(:,:,i),cross(w(:,:,i),rc(:,:,i)));
end

%backward recursion
f=zeros(3,1,n);
f(:,:,n)= f_n1 + m(n)*ac(:,:,n);
tau=zeros(3,1,n);
tau(:,:,n)= tau_n1 + cross(f_n1,rc(:,:,n)) - cross(f(:,:,n),R(:,:,n)'*Ai(1:3,4,n) + rc(:,:,n)) + I(:,:,n)*dw(:,:,n) + cross(w(:,:,n),I(:,:,n)*w(:,:,n));
for i=n-1:-1:1
    f(:,:,i)= R(:,:,i+1)*f(:,:,i+1) + m(i)*ac(:,:,i);
    tau(:,:,i)= R(:,:,i+1)*tau(:,:,i+1) + cross(R(:,:,i+1)*f(:,:,i+1),rc(:,:,i)) - cross(f(:,:,i),R(:,:,i)'*Ai(1:3,4,i) + rc(:,:,i)) + I(:,:,i)*dw(:,:,i) + cross(w(:,:,i),I(:,:,i)*w(:,:,i));
end

% projection
u=zeros(n,1);
for i=1:n
    u(i)=tau(:,:,i)'*R(3,:,i)';
end