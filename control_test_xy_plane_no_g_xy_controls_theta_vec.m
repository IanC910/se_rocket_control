
clear
close all

% Controller doesn't use the constant g (works for any gravity)
% Desired theta is calculated using the desired thrust vector
% Desired thrust is calculated from accel. feedback controls
% Do vector wise operations instead of components

g = 9.8;
sim_time    = 20;
dt          = 0.001;

num_steps   = sim_time / dt;
steps = 1 : num_steps;
t = steps * dt;

% Desired Conditions
x_ref = [10; -1000];

% Controller values
K_v         = 0.8;
K_a         = 10;
K_j         = 100;
K_omega     = 100;

max_omega   = 30 / 60 * 2 * pi;

% Initial Conditions
Z1          = zeros(1, num_steps);
Z2          = zeros(2, num_steps);

m           = 1;
x           = Z2;
v           = Z2;
a           = Z2;
F_thrust    = Z2;
theta       = Z1;
omega       = Z1;
alpha       = Z1;

% Start of sim

F_g = m * g;

for n = steps
    p           = max([n - 1, 1]);
    x0          = x(:, p);
    v0          = v(:, p);
    a0          = a(:, p);
    theta0      = theta(p);
    omega0      = omega(p);
    F_thrust0   = F_thrust(:, p);

    % Control
    v_desired = -K_v * (x0 - x_ref);

    a_desired = -K_a * (v0 - v_desired);

    j_desired = -K_j * (a0 - a_desired);
    F_thrust_dot_desired = m * j_desired;

    F_thrust_in = F_thrust0 + F_thrust_dot_desired * dt;
    F_thrust(:, n) = F_thrust_in;

    theta_in    = atan2(-F_thrust_in(1), F_thrust_in(2));

    omega_in    = -K_omega * (theta0 - theta_in);
    omega_in    = min([omega_in, max_omega]);
    omega_in    = max([omega_in, -max_omega]);
    omega(n)    = omega_in;

    % Update state
    a(:, n) = (F_thrust_in - F_g) / m;

    v(:, n) = v0 + a(:, n) * dt;

    x(:, n) = x0 + v0 * dt + 0.5 * a(:, n) * dt * dt;
    
    theta(n) = theta0 + omega_in * dt;
end

figure
tiledlayout(7, 1)

nexttile
plot(t, x(1, :))
title('x_1')

nexttile
plot(t, x(2, :))
title('x_2')

nexttile
plot(t, v(1, :))
title('v_1')

nexttile
plot(t, v(2, :))
title('v_2')

nexttile
plot(t, theta * 180 / pi)
title('theta in deg')

nexttile
plot(t, omega * 180 / pi)
title('omega in deg/s')

nexttile
plot(t, F_thrust)
title('F\_th')
hold off


figure
plot(x(1, :), x(2, :))
title('path')
