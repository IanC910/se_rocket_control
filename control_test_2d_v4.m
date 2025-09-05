
clear
close all

% Controller doesn't use the constant g (works for any gravity)
% Desired theta is calculated using the desired thrust vector
% Desired thrust is calculated from acceleration feedback controls
% Thrust input strength is calculated with current angle
% Velocity is capped

g = 9.8;
sim_time    = 60;
dt          = 0.001;

num_steps   = sim_time / dt;
steps = 1 : num_steps;
t = steps * dt;

% Desired Conditions
x_ref = [100; 1000];

% Controller values
K_v         = 0.5;
K_a         = 2;
K_j         = 40;
K_omega     = 80;

max_omega   = 30 / 60 * 2 * pi;

% Initial Conditions
Z1 = zeros(1, num_steps);
Z2 = zeros(2, num_steps);
m = 1;
F_g = [0; -m * g];

x               = Z2;
v               = Z2;
a               = Z2;
F_thrust_mag    = Z1;
theta           = Z1;
omega           = Z1;
alpha           = Z1;

a(:, 1) = F_g / m;
F_thrust_mag(1) = m * g;
theta(1) = pi / 2;

% Start of sim

for n = steps
    p           = max([n - 1, 1]);
    x0          = x(:, p);
    v0          = v(:, p);
    a0          = a(:, p);
    F_thrust0   = F_thrust_mag(p);
    theta0      = theta(p);
    dir0        = [cos(theta0); sin(theta0)];
    omega0      = omega(p);

    % Control
    v_desired = -K_v * (x0 - x_ref);
    v_desired_mag = norm(v_desired);
    v_desired_mag = min(v_desired_mag, 100);
    v_desired = v_desired_mag * v_desired / norm(v_desired);

    a_desired = -K_a * (v0 - v_desired);

    j_desired = -K_j * (a0 - a_desired);
    F_thrust_dot_desired = m * j_desired;

    F_thrust_desired = F_thrust0 * dir0 + F_thrust_dot_desired * dt;
    F_thrust_mag_in = dot(F_thrust_desired, dir0);
    F_thrust_mag_in = max(F_thrust_mag_in, 0);
    F_thrust_mag_in = min(F_thrust_mag_in, 100);

    theta_desired    = atan2(F_thrust_desired(2), F_thrust_desired(1));
    theta_distance   = theta0 - theta_desired;
    if(abs(theta_distance + 2 * pi) < abs(theta_distance))
        theta_distance = theta_distance + 2 * pi;
    end
    if(abs(theta_distance - 2 * pi) < abs(theta_distance))
        theta_distance = theta_distance - 2 * pi;
    end

    omega_in    = -K_omega * theta_distance;
    omega_in    = min([omega_in, max_omega]);
    omega_in    = max([omega_in, -max_omega]);

    % Update state
    a(:, n) = (F_thrust_mag_in * dir0 + F_g) / m;
    v(:, n) = v0 + a(:, n) * dt;
    x(:, n) = x0 + v0 * dt + 0.5 * a(:, n) * dt * dt;
    F_thrust_mag(n) = F_thrust_mag_in;
    theta(n) = theta0 + omega_in * dt;
    omega(n) = omega_in;
end

x_limits = [min([x(1, :), x(2, :)]), max([x(1, :), x(2, :)])];

figure
tiledlayout(7, 1)

nexttile
plot(t, x(1, :))
title('x_1')
axis([0, sim_time, x_limits])

nexttile
plot(t, x(2, :))
title('x_2')
axis([0, sim_time, x_limits])

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
plot(t, F_thrust_mag)
title('F\_thrust_mag')
hold off

figure
title('path')
limits = [x_limits, x_limits];
hold on
for s = 0 : sim_time - 1
    plot(x(1, s/dt + 1: (s + 1)/dt), x(2, s/dt + 1: (s + 1)/dt))
    axis(limits)
end
hold off
