function x = dt_dynamics(f,x0,u,dt)
% given the function f, it integrates it with Runge Kutta of the 4th order
% with the given time step dt and the initial condition x0
x = x0;
M=10; %steps per interval
DT=dt/M;
for j=1:M
   k1 = f(x, u);
   k2 = f(x + DT/2 * k1, u);
   k3 = f(x + DT/2 * k2, u);
   k4 = f(x + DT * k3, u);
   x=x+DT/6*(k1 +2*k2 +2*k3 + k4);
end
end