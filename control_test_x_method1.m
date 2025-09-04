
clear
close all

g = 9.8;
sim_time    = 10;
dt          = 0.001;

num_steps   = sim_time / dt;
steps = 1 : num_steps;
t = steps * dt;

% Desired Conditions
x_r = 10;

% Controller values
K_v         = 1;
K_a         = 2.5;
K_omega     = 10;

max_theta   = pi / 4;
max_omega   = 30 / 60 * 2 * pi;

% Initial Conditions
m = 1;
x       = zeros(size(steps));
v_x     = zeros(size(steps));
theta   = zeros(size(steps));
omega   = zeros(size(steps));

% Start of sim

F_g = m * g;

for n = steps
    p = max([n - 1, 1]);
    x0 = x(p);
    v_x0 = v_x(p);
    theta0 = theta(p);

    % Calculate inputs
    v_x_in      = -K_v * (x0 - x_r);

    a_x_in      = -K_a * (v_x0 - v_x_in);

    theta_in    = atan(a_x_in / g);
    theta_in    = min([theta_in, max_theta]);
    theta_in    = max([theta_in, -max_theta]);

    omega_in    = -K_omega * (theta0 - theta_in);
    omega_in    = min([omega_in, max_omega]);
    omega_in    = max([omega_in, -max_omega]);
    
    F_thrust = F_g / cos(theta0);
    F_thrust_x = F_thrust * sin(theta0);

    % Update state
    a_x = F_thrust_x / m;
    v_x(n) = v_x0 + a_x * dt;
    x(n) = x0 + v_x0 * dt + 0.5 * a_x * dt * dt;
    theta(n) = theta0 + omega_in * dt;
    omega(n) = omega_in;
end

figure
plot(t, x)
title('x')

figure
plot(t, v_x)
title('v')

figure
plot(t, theta * 180 / pi)
title('theta in deg')

figure
plot(t, omega * 180 / pi)
title('omega in deg/s')
