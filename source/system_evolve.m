clear all
close all
clc
x_0 = rand(6,1);
u = rand(3,1);
Ts = 0.01;
x_by_hand = dt_dynamics(@ct_dynamics,x_0,u,Ts);

%ode45 has variable stepsize, but we can extract the values only at the
%points we want by passing not a [t0 tf] but a set of points, like here
%check https://it.mathworks.com/matlabcentral/answers/92961-how-do-i-use-a-fixed-step-size-with-ode23-and-ode45-in-matlab
[~,x_auto]=ode45(@(t,x)ct_dynamics(x,u),[0 Ts 2*Ts],x_0);
disp(x_by_hand')
disp(x_auto(2,:))

%or pass a span from 0 to Ts and take the last sample
[~,x_auto2]=ode45(@(t,x)ct_dynamics(x,u),[0 Ts],x_0);
disp(x_auto2(end,:))