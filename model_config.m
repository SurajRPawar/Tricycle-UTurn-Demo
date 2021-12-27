% -----------------------------------------------------
% model_config.m
% This M-function acquires the configuration of the robot
% Programmed by Tomonari Furukawa for MTRN9224
% March 11, 2004
% 
% -----------------------------------------------------
function [xb, yb, xfw, yfw, xrlw, yrlw, xrrw, yrrw] = model_config(z, u)

global L;
global d;
global R_w;

x = z(1);  % Length of the vehicle
y = z(2);
th = z(3);

st = u(1);

xfc = x + L*cos(th);
yfc = y + L*sin(th);

xfl = xfc - 0.5*d*sin(th);
yfl = yfc + 0.5*d*cos(th);
xfr = xfc + 0.5*d*sin(th);
yfr = yfc - 0.5*d*cos(th);
xrl = x - 0.5*d*sin(th);
yrl = y + 0.5*d*cos(th);
xrr = x + 0.5*d*sin(th);
yrr = y - 0.5*d*cos(th);

xfwf = xfc + R_w*cos(th+st);
yfwf = yfc + R_w*sin(th+st);
xfwr = xfc - R_w*cos(th+st);
yfwr = yfc - R_w*sin(th+st);

xrlwf = xrl + R_w*cos(th);
yrlwf = yrl + R_w*sin(th);
xrlwr = xrl - R_w*cos(th);
yrlwr = yrl - R_w*sin(th);

xrrwf = xrr + R_w*cos(th);
yrrwf = yrr + R_w*sin(th);
xrrwr = xrr - R_w*cos(th);
yrrwr = yrr - R_w*sin(th);

zfc = [xfc, yfc];

xb = [xfl, xfr, xrr, xrl, xfl];
yb = [yfl, yfr, yrr, yrl, yfl];

xfw = [xfwf, xfwr];
yfw = [yfwf, yfwr];

xrlw = [xrlwf, xrlwr];
yrlw = [yrlwf, yrlwr];

xrrw = [xrrwf, xrrwr];
yrrw = [yrrwf, yrrwr];

    