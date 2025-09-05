
clear
close all

g = 9.8;
sim_time    = 20;
dt          = 0.001;

num_steps   = sim_time / dt;
steps = 1 : num_steps;
t = steps * dt;

% Desired Conditions
x_r = 10;
y_r = -1000;

% Controller values
K_v         = 0.8;
K_a         = 10;
K_omega     = 100;
K_j         = 100;

max_theta   = pi / 4;
max_omega   = 30 / 60 * 2 * pi;

% Initial Conditions
m = 10;
Z       = zeros(size(steps));

x       = Z;
y       = Z;
v_x     = Z;
v_y     = Z;
a_y     = -g;
theta   = Z;
omega   = Z;
F_th    = Z;

% Start of sim

F_g = m * g;

for n = steps
    p = max([n - 1, 1]);
    x0      = x(p);
    y0      = y(p);
    v_x0    = v_x(p);
    v_y0    = v_y(p);
    theta0  = theta(p);
    F_th0   = F_th(p);

    % Control
    v_x_in      = -K_v * (x0 - x_r);
    v_y_in      = -K_v * (y0 - y_r);

    a_x_in      = -K_a * (v_x0 - v_x_in);
    a_y_in      = -K_a * (v_y0 - v_y_in);

    theta_in    = atan(-a_x_in / g);
    theta_in    = min([theta_in, max_theta]);
    theta_in    = max([theta_in, -max_theta]);

    omega_in    = -K_omega * (theta0 - theta_in);
    omega_in    = min([omega_in, max_omega]);
    omega_in    = max([omega_in, -max_omega]);
    
    F_th_y_dot = -K_j * m * (a_y - a_y_in);
    F_th_dot = F_th_y_dot / cos(theta0);

    F_th_in = F_th0 + F_th_dot * dt;
    F_th_in = max([F_th_in, 0]);
    F_th(n) = F_th_in;

    % Update state
    a_x = F_th_in * -sin(theta0) / m;
    a_y = (F_th_in * cos(theta0) - F_g) / m;

    v_x(n) = v_x0 + a_x * dt;
    v_y(n) = v_y0 + a_y * dt;

    x(n) = x0 + v_x0 * dt + 0.5 * a_x * dt * dt;
    y(n) = y0 + v_y0 * dt + 0.5 * a_y * dt * dt;
    
    theta(n) = theta0 + omega_in * dt;
    omega(n) = omega_in;
end

figure
plot(t, x)
title('x')

figure
plot(t, y)
title('y')

figure
plot(x, y)
title('path')

figure
plot(t, v_x)
title('v_x')

figure
plot(t, v_y)
title('v_y')

figure
plot(t, theta * 180 / pi)
title('theta in deg')

figure
plot(t, omega * 180 / pi)
title('omega in deg/s')

figure
plot(t, F_th)
title('F\_th')
