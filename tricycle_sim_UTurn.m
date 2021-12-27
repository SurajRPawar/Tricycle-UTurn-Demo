% -------------------------------------------------------------------------
% Simulation of a tricycle taking a U Turn
% -------------------------------------------------------------------------
%
% State feedback applied to a kinematic model of the tricycle. Note that
% the processing of writing the animation is very slow. Feel free to
% optimize it through better coding.
%
% Asssumptions
% 1. Use small angle approximation
% 2. Tricycle has constant forward velocity of v (m/s)
% 3. All the states of the model (Y coordinate and orientation angle) are
%    measured and avaiable.
% 4. The U-Turn is made up of linear segments, and we know the coordinates
%    of each segment in the global frame of reference.
%
% Disclaimer : This code is not meant to demonstrate a practical or
% implementable lane following controller. The goal of this assignment was
% to get a feel of first principles modeling, and to try to develop a
% controller for lane following. Systems in actual vehicles are more
% sophisticated than this, however this exercise was for me to work on a
% "toy problem" and challenge myself to come up with a solution from
%
% Suraj R Pawar, 12-27-2021
% -------------------------------------------------------------------------

clear all; close all; clc;

global L d R_w v k1 k2 delta_max xtrack ytrack

%% All parameters

% Vehicle parameters
L = 0.151;       % Length between the front wheel axis and rear wheel axis (m)
d = 0.14;        % Distance between the rear wheels (m)
R_w = 0.025;     % Radius of wheel (m)
delta_max = 0.15;% Maximum steering angle, (rad)
v = 1;           % Forward velocity of tricycle, (m/s)

% Define track
xd1 = [0.0 2.0 10.0 14.0 15.0]; % Coordinates of bottom half
yd1 = [0 0 3.0 10.0 15.0];      % Coordinates of bottom half
xd22 = [0 2.0 10.0 14.0];       % Coordinates of upper half
yd22 = [30.0 30.0 27.0 20.0];   % Coordinates of upper half
xd2 = fliplr(xd22);
yd2 = fliplr(yd22);

xtrack = [xd1,xd2];
ytrack = [yd1,yd2];

% Desired poles
poles = [-20, -0.2];

% Simulation time
t0 = 0;         % Simulation start time, (s)
dt = 0.001;     % Time step, (s)
t_max = 47;     % Simulation end time (s)

% Design via pole placement
A_mat = [0,v;0,0]; % Linear A matrix
B_mat = [0;v/L];   % Linear B matrix
Kgains = place(A_mat,B_mat,poles); % Find gains [k1 k2] to apply to x =[y psi]
k1 = Kgains(1)
k2 = Kgains(2)

% Save animation to file ?
save_anim = 0; % 0 = don't save, 1 = save
show_anim = 0; % To show animation or not
filename = 'Tricycle Lane Keeping.avi';

%% Initial conditions
x0 = 0;         % Initial x position, m
y0 = 1;         % Initial y position, m
psi_deg0 = 10;  % Initial orientation, rad
z0 = [x0;y0;psi_deg0*pi/180];   % Initial state vector

%% Simulation
n = floor(t_max/dt);    % Number of steps

% RK4 simulation
[t y] = rk4fixed('tricycle_model_UTurn',[t0 t_max],z0,n);

%% Gather model outputs
deltaout = zeros(1,size(y,1));
erroryout = zeros(1,size(y,1));
errorpsiout = zeros(1,size(y,1));
desposout = zeros(2,size(y,1));
despsiout = zeros(1,size(y,1));
for i=1:size(y,1)
    [zout yout] = tricycle_model_UTurn(t(i),y(i,:));
    deltaout(i) = yout(1);
    erroryout(i) = yout(2);
    errorpsiout(i) = yout(3);
    desposout(:,i) = yout(4:5);
    despsiout(i) = yout(6);
end

%% Plot everything
figure(1); % Trajectory of vehicle's CG
hold on;
plot(desposout(1,:),desposout(2,:),'--r',y(:,1),y(:,2)),...
    title('CG trajectory'),xlabel('X position (m)'),ylabel('Y position (m)')...
    ,legend('Track','CG Traj')

figure(2); % Delta, Psi, Error vs time
subplot(4,1,1),plot(t,deltaout),title('Steering Angle'),...
    xlabel('Time (s)'),ylabel('Steer Angle (rad)')
subplot(4,1,2),plot(t,erroryout),title('Lateral position error'),...
    xlabel('Time (s)'),ylabel('Error (m)')
subplot(4,1,3),plot(t,y(:,3)),title('Orientation'),...
    xlabel('Time (s)'),ylabel('Psi (rad)')
subplot(4,1,4),plot(t,errorpsiout),title('Orientation error'),...
    xlabel('Time (s)'),ylabel('Psi error (rad)')

%% Animation
if show_anim == 1
    if save_anim == 1
        myVideo = VideoWriter(filename);
    end

    % Gather vehicle configuration
    [xb, yb, xfw, yfw, xrlw, yrlw, xrrw, yrrw] = model_config(z0, psi_deg0*pi/180);
    anim = figure(4);
    hold on;
    % Plot tricycle and define plot id
    plot(xtrack,ytrack,'--k'),title('Tricycle lane keeping - state feedback');
    plotzb = plot(xb, yb);  % Plot robot base
    plotcg = plot(z0(1), z0(2),'go'); % Plot CG of tricycle
    plotdes = plot(desposout(1,1), desposout(2,1),'rx'); % Plot desired location on UTurn
    plotzfw = plot(xfw, yfw, 'r');  % Plot front wheel
    plotzrlw = plot(xrlw, yrlw, 'r');  % Plot rear left wheel
    plotzrrw = plot(xrrw, yrrw, 'r');  % Plot rear right wheel
    ax=gca; axis fill;
    axis([-1 2 -1 2]);
    moving_x = zeros(1,500);   % These points will help us get a moving window for the x scale
    moving_y = zeros(1,500);   % These points will help us get a moving window for the y scale

    %Beginning of simulation
    if save_anim == 1
        open(myVideo);       
    end

    for i = 2:size(y,1)
        [xb, yb, xfw, yfw, xrlw, yrlw, xrrw, yrrw] = model_config(y(i,:), deltaout(i));

        r = rem(i,500);            
        if r == 0                   
            r = 500;
        end                         
        moving_x(1,r) = y(i,1);  % This code fills points up into our moving window of points.
        moving_y(1,r) = y(i,2);
        ax.XLim = [min(moving_x)-1 max(moving_x)+2];   % Adjust viewing window
        ax.YLim = [min(moving_y)-1 max(moving_y)+2];
        set(plotcg,'xdata',y(i,1));
        set(plotcg,'ydata',y(i,2));
        set(plotdes,'xdata',desposout(1,i));
        set(plotdes,'ydata',desposout(2,i));

        set(plotzb,'xdata',xb);
        set(plotzb,'ydata',yb);
        set(plotzfw,'xdata',xfw);
        set(plotzfw,'ydata',yfw);
        set(plotzrlw,'xdata',xrlw);
        set(plotzrlw,'ydata',yrlw);
        set(plotzrrw,'xdata',xrrw);
        set(plotzrrw,'ydata',yrrw);
        pause(0);  % Pause by 0.2s for slower simulation
        if rem(i,50)==0 && save_anim == 1 % Change `50' to adjust how many frames are written
            writeVideo(myVideo,getframe(anim));
        end
    end

    if save_anim == 1
        close(myVideo);
    end
    fprintf('Animation written to %s \n', filename);
end