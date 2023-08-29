function u = modNE(dq,v,ddq,m,I,A,pc,g0)
% Implementation of Modified Newton-Euler algorithm
% Input:
%     - dq: joint velocity used to compute C(q,dq)
%     - v: velocity to multiply by the factorization matrix, i.e., C(q,dq)v
%     - ddq: joint acceleration
%     - m: mass vector
%     - I: 3x3xn tensor of inertia matrices
%     - A: 4x4xn tensor of homogeneous transformations between consecutive frames
%     - pc: 3xn matrix with the i-th column being the CoM of the i-th link, expressed in the i-th frame
%     - g0: scalar gravity acceleration
% Output:
%     - inverse dynamics: u=M(q)ddq+C(q,dq)v+g(q)

n = size(dq,1);
% check dimension consistency
if ~(size(v,1)==n && size(ddq,1)==n && size(dq,2)==1 && ...
     size(v,2)==1 && size(ddq,2)==1 && size(m,1)==n && ...
     size(I,3)==n && size(A,3)==n && size(pc,2)==n) 
    disp("Inconsistent input arguments dimension")
    u = zeros(n,1);
    return
end

% initialization
R = A(1:3,1:3,:);
p = [R(:,:,1)'*A(1:3,4,1) R(:,:,2)'*A(1:3,4,2) R(:,:,3)'*A(1:3,4,3)];

if isa(A,'sym')
    w = sym(zeros(3,n+1));
    wa = sym(zeros(3,n+1));
    dw = sym(zeros(3,n+1));
    a = sym(zeros(3,n+1))-[[0 0 -g0]' sym(zeros(3,n))];
    ac = sym(zeros(3,n+1));
    f = sym(zeros(3,n+1));
    t = sym(zeros(3,n+1));
    u = sym(zeros(n,1));
else
    w = zeros(3,n+1);
    wa = zeros(3,n+1);
    dw = zeros(3,n+1);
    a = zeros(3,n+1)-[[0 0 -g0]' zeros(3,n)];
    ac = zeros(3,n+1);
    f = zeros(3,n+1);
    t = zeros(3,n+1);
    u = zeros(n,1);
end

% forward recursion
for i = 2:n+1
    w(:,i) = R(:,:,i-1)'*(w(:,i-1)+dq(i-1)*[0 0 1]');
    wa(:,i) = R(:,:,i-1)'*(wa(:,i-1)+v(i-1)*[0 0 1]');
    dw(:,i) = R(:,:,i-1)'*(dw(:,i-1)+ddq(i-1)*[0 0 1]'+cross(dq(i-1)*wa(:,i-1),[0 0 1]'));
    a(:,i) = R(:,:,i-1)'*a(:,i-1)+cross(dw(:,i),p(:,i-1))+cross(w(:,i),cross(wa(:,i),p(:,i-1)));
    ac(:,i) = a(:,i)+cross(dw(:,i),pc(:,i-1))+cross(w(:,i),cross(wa(:,i),pc(:,i-1)));
end

w = w(:,2:n+1);
wa = wa(:,2:n+1);
dw = dw(:,2:n+1);
ac = ac(:,2:n+1);
R = cat(3,R,eye(3));

% backward recursion
for i = n:-1:1
    f(:,i) = R(:,:,i+1)*f(:,i+1)+m(i)*ac(:,i);
    t(:,i) = R(:,:,i+1)*t(:,i+1)+cross(R(:,:,i+1)*f(:,i+1),pc(:,i))-cross(f(:,i),p(:,i)+pc(:,i))+I(:,:,i)*dw(:,i)+cross(wa(:,i),I(:,:,i)*w(:,i));
    % assuming all revolute joints
    u(i) = t(:,i)'*(R(:,:,i)'*[0 0 1]');
end
end