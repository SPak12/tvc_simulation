%% ME 135 TVC Control
Ft = 300; % Thrust [N]
x_cm = 1.2; % COM distance [m]
I = 3.5; % Moment of inertia [kg m^2]
m = 3; % Mass [kg]
g = 9.81; % Gravitational acceleration [kg/s^2]
theta_d = 0; % Disturbance angle
theta_max = 30; % Maximum gimbal angle
theta_des = 0; % Desired rocket orientation
theta_0 = 15; % Initial rocket orientation

Kp = 40; % Proportional gain
Kd = 5; % Derivative gain
Ki = 0; % Integral gain

sim('tvc_sim.slx'); % Nonlinear model
% sim('tvc_linear.slx'); % Linear approximation
t = tout;
theta_g = rad2deg(u.Data);
theta_r = rad2deg(y.Data);

figure
yline(theta_des, '--k'); hold on
plot(t, theta_g, 'b', t, theta_r, 'r')
title(['PID Controller, K_p = ', num2str(Kp), ', K_i = ', num2str(Ki), ...
    ', K_d = ', num2str(Kd)])
xlabel('Time [s]')
ylabel('Output [deg]')
legend('\theta_r (Desired Orientation)', '\theta_g (Gimbal Angle)', '\theta_r (Rocket Orientation)')
