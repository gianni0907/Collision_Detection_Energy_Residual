clear all
close all
clc
q2_l=-pi; q2_u=pi;
q3_l=-pi; q3_u=pi;
dq_l=ones(3,1)*-2; dq_u=ones(3,1)*2;
%x is [q2,q3,dq1,dq2,dq3]
x0=[pi/4,pi/4,1,1,1]';
% tried with: active-set, sqp, interior point and always the same result.
% tried with rand(5,1) as x0 and still the same result.
sol=fmincon(@(x) -norm_c(x(1),x(2),x(3),x(4),x(5))/norm(x(3:5)), x0,...
                [], [], [], [], [q2_l;q3_l;dq_l], [q2_u;q3_u;dq_u]);
upper_bound = norm_c(sol(1),sol(2),sol(3),sol(4),sol(5))/norm(sol(3:5))