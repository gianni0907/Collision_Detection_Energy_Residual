clear all
close all
clc
x_0 = rand(6,1);
u = rand(3,1);
Ts = 0.01;
x_by_hand = dt_dynamics(@ct_dynamics,x_0,u,Ts);
disp('*** considering one timestep Ts as integration horizon ***')
%ode45 has variable stepsize, but we can extract the values only at the
%points we want by passing not a [t0 tf] but a set of points, like here
%check https://it.mathworks.com/matlabcentral/answers/92961-how-do-i-use-a-fixed-step-size-with-ode23-and-ode45-in-matlab
[~,x_auto]=ode45(@(t,x)ct_dynamics(x,u),[0 Ts 2*Ts],x_0);
disp(x_by_hand')
disp(x_auto(2,:))

%or pass a span from 0 to Ts and take the last sample
[~,x_auto2]=ode45(@(t,x)ct_dynamics(x,u),[0 Ts],x_0);
disp(x_auto2(end,:))

%% what happens when we recursively apply our dt_dynamics for a total time T?
% Is it different from using ode45 with tspan [0 T]? And what if we
% recursively apply ode45? Let's see

T = 2;
disp('*** considering a longer horizon ***')
% recursively apply our dt_dynamics
x_by_hand = x_0;
for i=1:T/Ts
    x_by_hand = dt_dynamics(@ct_dynamics,x_by_hand,u,Ts);
end
disp(x_by_hand')

% apply ode45 oneshot
[~,x_total]=ode45(@(t,x)ct_dynamics(x,u),[0 T],x_0);
disp(x_total(end,:))

% apply ode45 recursively
x_steps=x_0;
for i=1:T/Ts
    [~,x_steps]=ode45(@(t,x)ct_dynamics(x,u),[0 Ts],x_steps);
    x_steps = x_steps(end,:);
end
disp(x_total(end,:))
% conclusion: applying ode45 recursively or oneshot is the same, but they
% are both different from applying our dt_dynamics. The larger T, the larger the effect