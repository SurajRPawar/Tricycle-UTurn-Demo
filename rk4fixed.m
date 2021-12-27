function [T,X]=rk4fixed(Fcn,Tspan,X0,N)
% Matlab implementation of a fixed-step RK4 algorithm.
% Written by R.G. Longoria 950129.
% This version was revised July 1998 to be compatible with
% the Matlab ODE suite.
% Usage: >> [T,Y] = rk4fixed('Fcn',[T0 Tf],X0,N);
% Inputs to program:
% X0 = initial value(s) of the dependent variable.
%      Example: X0=[1;1];
% Tspan = vector, [T0 Tf]
% T0 = starting value of the independent variable (e.g. time)
% Tf = ending value of the independent variable
% N = number of steps
% Fcn = the name of the function that defines the ODEs, f(t,x).
%     Note: User needs to define this function in an m-file:
%           function yprime=Fcn(T,X)
% Computed in rk4fixed:
% h = fixed step size = (Tspan(2)-Tspan(1))/N
% Output from rk4fixed:
% Two arrays: X and T over the time period specified.
%
% Here is an example of the use of this program from\\
% the command line:
%      [T,X]=rk4fixed(`myfunction',[0 0.5],[0.1;0.0],500);
%      This integrates 2 eqs specified in `myfunction' from
%      time 0 to 0.5 with steps of h=(0.5-0)/500.  The initial
%      conditions are given by [0.1;0.0].
%
h = (Tspan(2)-Tspan(1))/N;
halfh = 0.5*h;
%
neqs=size(X0);
X=zeros(neqs(1),N);
T=zeros(1,N);
X(:,1)=X0;
T(1)=Tspan(1);
Td = Tspan(1);
Xd = X0;
for i=2:N,
RK1 = feval(Fcn,Td,Xd);
Thalf = Td + halfh;
Xtemp = Xd + halfh*RK1;
RK2 = feval(Fcn,Thalf,Xtemp);
Xtemp = Xd + halfh*RK2;
RK3 = feval(Fcn,Thalf,Xtemp);
Tfull = Td + h;
Xtemp = Xd + h*RK3;
RK4 = feval(Fcn,Tfull,Xtemp);
X(:,i) = Xd + h*(RK1+2.0*(RK2+RK3)+RK4)/6;
T(i) = Tfull;
Xd = X(:,i);
Td = T(i);
end
X=X';T=T';