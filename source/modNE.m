function u = modNE(dq,v,ddq,m,I,A,pc,g0)
% Implementation of Modified Newton-Euler algorithm
if ~exist('g0','var')
     % no gravity acceleration parameter, default it to 0
      g0 = 0;
end

n = size(dq,1);
% check dimension consistency
if ~(size(v,1)==n && size(ddq,1)==n && size(dq,2)==1 && ...
     size(v,2)==1 && size(ddq,2)==1 && size(m,1)==n && ...
     size(I,3)==n && size(A,3)==n && size(pc,2)==n) 
    disp("Inconsistent input arguments dimension")
    return
end

% initialization
R = A(1:3,1:3,:);
p = [R(:,:,1)'*A(1:3,4,1) R(:,:,2)'*A(1:3,4,2) R(:,:,3)'*A(1:3,4,3)];
w = zeros(3,n+1);
wa = zeros(3,n+1);
dw = zeros(3,n+1);
a = zeros(3,n+1)-[[0 0 -g0]' zeros(3,n)];
ac = zeros(3,n+1);
f = zeros(3,n+1);
t = zeros(3,n+1);
u = zeros(n,1);

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
    t(:,i) = R(:,:,i+1)*t(:,i+1)+cross(R(:,:,i+1)*f(:,i+1),pc(:,i))-cross(f(:,i),p(:,i)+pc(:,i))+I(:,:,i)*dw(:,i)+cross(w(:,i),I(:,:,i)*wa(:,i));
    % assuming all revolute joints
    u(i) = t(:,i)'*(R(:,:,i)'*[0 0 1]');
end
end