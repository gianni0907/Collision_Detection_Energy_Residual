function q = inv_kin(p)
q = [0,0,0]';
L1=0.5; % link lengths [m]  
L2=0.5;
L3=0.4;
c3 = (p(1)^2+p(2)^2+(p(3)-L1)^2-L2^2-L3^2)/(2*L2*L3);

q(3)=-atan2(sqrt(1-c3^2),c3);

if p(1)^2+p(2)^2 >0
    q(1)=atan2(p(2),p(1));
end
if p(1)^2+p(2)^2+(p(3)-L1)^2 ~= 0
    sol=linsolve([L2+L3*cos(q(3)) -L3*sin(q(3)); L3*sin(q(3)) L2+L3*cos(q(3))], ...
            [cos(q(1))*p(1)+sin(q(1))*p(2);p(3)-L1]);
    q(2)=atan2(sol(2),sol(1));
end
end