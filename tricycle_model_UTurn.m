function [zdot y] = tricycle_model(t,z)
%-------------------------------------------------------------------------
% Vehicle Systems Dynamics and Controls (ME390) - UT Austin, Spring 2018
% Tricycle model
%
% Inputs
% 1. t : current time (s)
% 2. z : states of the kinematic model [y; psi]
%
% Outputs
% 1. delta : turning angle (rad)
% 2. e_y : error in y coordinate (m)
% 3. e_psi : error in orientation (rad)
% 4. des_pos(1) : desired x coordinate (m)
% 5. des_pos(2) : desired y coordinate (m)
% 6. des_psi : desired orientation (rad)
%
% Suraj R Pawar, 12-27-2021
%-------------------------------------------------------------------------

%% Access global variables
global L v k1 k2 delta_max xtrack ytrack
% L is length between the front wheel axis and rear wheel axis [m]
% v is speed, m/s
% k1, k2 are feedback control gains
% xtrack, ytrack are the UTurn coordinates in global frame

%% State variables
x = z(1);
y = z(2);
psi = z(3);

%% Update desired position on track

persistent A B track_index t_marker base_norm des_pos
% A is the start goal in a linear segment of the track
% B is the end goal in the linear segment of the track
% track_index is a counter to keep goals shifting through time

% Start condition, set A and B

if t ==0
    t_marker = 0;
    track_index = 1;
    A = [xtrack(track_index);ytrack(track_index)];
    B = [xtrack(track_index+1);ytrack(track_index+1)];
    base_norm = norm(B-A);
    des_pos = [0;0];
    des_psi = 0;
end

if norm(des_pos-A)>= base_norm
    % Update A and B
    track_index = track_index+1;
    t_marker=t-(norm(des_pos-B)/v);
    A = [xtrack(track_index);ytrack(track_index)];
    B = [xtrack(track_index+1); ytrack(track_index+1)];
    base_norm = norm(B-A);
end
des_pos = A + v*(t-t_marker)*(B-A)./norm(B-A);
des_psi = atan2((B(2)-A(2)),(B(1)-A(1)));

%% Calculate error
e_psi = psi-des_psi;
dev_vec = des_pos - [x;y];
e_y = y - des_pos(2);

%% Apply control law
delta = -k1*e_y-k2*e_psi;

% Apply saturation limits to delta
if abs(delta)>=delta_max
    delta = sign(delta)*delta_max;
end

%% State equations
xdot = v * cos(psi);
ydot = v * sin(psi);
psidot = (v/L)*tan(delta);

%% Outputs
zdot = [xdot;ydot;psidot];
y(1) = delta;
y(2) = e_y;
y(3) = e_psi;
y(4) = des_pos(1);
y(5) = des_pos(2);
y(6) = des_psi;