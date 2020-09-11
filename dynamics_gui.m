%% ME 135 TVC Dynamics Function for GUI
% -----Inputs-----
% dt: Time step
% p0: Initial position [x0 y0 z0]
% v0: Initial velocity [v_x0 v_y0 v_z0]
% theta_r0: Initial rocket orientation in both xz- and yz-planes
%           [theta_rx0 theta_ry0]
% theta_g0: Initial angle of gimbal in both xz- and yz-planes
%           [theta_g0x theta_g0y]
% omega_0: Initial angular velocity in both xz- and yz-planes
%          [omega_x0 omega_y0]
% disturbances: Disturbance forces in the x- and y-direction [d_x d_y] [N]

% -----Outputs-----
% p: New position [x y z]
% v: New velocity [v_x v_y v_z]
% a: New acceleration [a_x a_y a_z]
% theta_g: New gimbal angle in both xz- and yz-planes [theta_gx theta_gy]
% theta_r: New rocket orientation in both xz- and yz-planes
%          [theta_rx theta_ry]
% omega: New angular velocity in both xz- and yz-planes [omega_x omega_y]

function [p, v, a, theta_r, theta_g, error, omega] = dynamics_gui(dt, ...
    p0, v0, theta_g0, theta_r0, prev_error, error_i, omega_0, disturbances)
%% Initial parameters
Ft = 300; % Thrust force [N]
m = 3; % Mass [kg]
g = 9.81; % Gravitational acceleration [kg/s^2]
x_cm = 1.2; % Distance of COM from rocket's base [m]
h = 1.8288; % Height of rocket in meters
I = 3.5; % Moment of inertia about COM [kg m^2]

Kd = 60; Kp = 20; Ki = 0; % PID controller gains

%% Splitting input vectors into easier to use variables
theta_rx0 = theta_r0(1);
theta_ry0 = theta_r0(2);

theta_gx0 = theta_g0(1);
theta_gy0 = theta_g0(2);

x0 = p0(1);
y0 = p0(2);
z0 = p0(3);

v_x0 = v0(1);
v_y0 = v0(2);
v_z0 = v0(3);

d_x = disturbances(1); %Disturbance in the x-direction
d_y = disturbances(2); %Disturbance in the y-direction

%% Acceleration calculations
theta_x = theta_rx0 + theta_gx0;
theta_y = theta_ry0 + theta_gy0;
a_x = (1/m)*(Ft*sind(theta_x)+d_x);
if theta_x>90
    a_x = (1/m)*(Ft*-1*sind(theta_x)+d_x);
end
a_y = (1/m)*(Ft*sind(theta_y)+d_y);
if theta_y>90
    a_y = (1/m)*(Ft*-1*sind(theta_y)+d_y);
end
a_z = Ft/m*cosd(theta_x)*cosd(theta_y) - g;
if theta_x>90 && theta_y>90
    a_z = -Ft/m*cosd(theta_x)*cosd(theta_y) - g;
end
a = [a_x a_y a_z];

%% Velocity Calculations
v_x = v_x0 + a_x*dt;
v_y = v_y0 + a_y*dt;
v_z = v_z0 + a_z*dt;
v = [v_x v_y v_z];

%% Position Calculations
x = x0 + v_x0*dt + 1/2*a_x*dt^2;
y = y0 + v_y0*dt + 1/2*a_y*dt^2;
z = z0 + v_z0*dt + 1/2*a_z*dt^2;
p = [x y z];

%% Rocket orientation calculation
% Euler's second law: M_C = Ft*x_cm*sin(pi - theta_g) = I*alpha
% Definition of phi, a 2D vector describing the two angles between the
% disturbance force and the vector from the CoM to the geometric center
phi = 90 - theta_r0;

alpha = -(1/I)*((Ft*x_cm)*sind(theta_g0) + disturbances.*(h - x_cm).*sind(phi));
omega = omega_0 + alpha*dt;
theta_r = theta_r0 + omega*dt;

%% Controls
error = theta_r;
error_d = (error - prev_error)/dt;
error_i = error_i + error;

theta_g = Kd*error + Kp*error_d + Ki*error_i;
for i = 1:2
    if theta_g(i) > 30
        theta_g(i) = 30;
    elseif theta_g(i) < -30
        theta_g(i) = -30;
    end
end

end
