function y=check_in_workspace(p,l)
c3 = (p(1)^2+p(2)^2+(p(3)-l(1))^2-l(2)^2-l(3)^2)/(2*l(2)*l(3));
if (abs(c3) > 1)
    disp("Point ["+compose("%.2f",p(1))+", "+compose("%.2f",p(2))+", "+...
        compose("%.2f",p(3))+"] out of workspace");
    y=0;
    return
end
y=1;
end