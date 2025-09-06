
clc
clear
close all

% Controller doesn't use the constant g (works for any gravity)
% Desired theta is calculated using the desired thrust vector
% Desired thrust is calculated from acceleration feedback controls
% Thrust input strength is calculated with current angle
% Velocity is capped
% Using ship basis matrix for finding desired rotation

g = 9.8;
sim_time    = 60;
dt          = 0.02;

num_steps   = sim_time / dt;
steps = 1 : num_steps;
t = steps * dt;

% Desired Conditions
x_ref = [100; 1000];

% Controller values
K_v         = 0.5;
K_a         = 2;
K_j         = 40;
K_omega     = 10;

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
B               = zeros(2, 2, num_steps); B(:, :, 1) = [0 -1; 1 0];
theta           = Z1;
omega_s         = Z1;

a(:, 1) = F_g / m;
F_thrust_mag(1) = m * g;
theta(1) = atan2(B(2, 1, 1), B(1, 1, 1));

% Start of sim

for n = steps
    p           = max([n - 1, 1]);
    x0          = x(:, p);
    v0          = v(:, p);
    a0          = a(:, p);
    F_thrust0   = F_thrust_mag(p);
    B0          = B(:, :, p);
    dir0        = B0(:, 1);
    theta0      = theta(p);

    % Control
    v_ref_uncapped = -K_v * (x0 - x_ref);
    v_ref_mag = min(norm(v_ref_uncapped), 100);
    v_ref = v_ref_mag * v_ref_uncapped / norm(v_ref_uncapped);

    a_ref = -K_a * (v0 - v_ref);

    j_ref = -K_j * (a0 - a_ref);
    F_thrust_dot_ref = m * j_ref;

    F_thrust_ref = F_thrust0 * dir0 + F_thrust_dot_ref * dt;
    F_thrust_mag_in_uncapped = dot(F_thrust_ref, dir0);
    F_thrust_mag_in = min(max(F_thrust_mag_in_uncapped, 0), 100);

    % Get thrust vector in ship coords
    F_thrust_ref_s = B0 \ F_thrust_ref;
    F_thrust_ref_dir_s = F_thrust_ref_s / norm(F_thrust_ref_s);
    b_1_s = [1; 0]; % Forward direction in ship coords

    % Delta theta in ship coords
    delta_theta_temp_s = cross([b_1_s; 0], [F_thrust_ref_dir_s; 0]);
    delta_theta_mag = asin(norm(delta_theta_temp_s));
    delta_theta_s = delta_theta_mag * sign(delta_theta_temp_s(3));

    omega_in_uncapped_s = K_omega * delta_theta_s;
    omega_in_mag = min(norm(omega_in_uncapped_s), max_omega);
    omega_in_s = omega_in_mag * omega_in_uncapped_s / norm(omega_in_uncapped_s);

    % Update state
    b1      = B0(:, 1);
    F_net   = F_thrust_mag_in * b1 + F_g;
    a(:, n) = F_net / m;
    v(:, n) = v0 + a(:, n) * dt;
    x(:, n) = x0 + v0 * dt + 0.5 * a(:, n) * dt * dt;
    F_thrust_mag(n) = F_thrust_mag_in;
    dtheta = omega_in_s * dt;
    B(:, :, n) = B0 * [cos(dtheta) -sin(dtheta); sin(dtheta), cos(dtheta)];
    theta(n) = theta0 + dtheta;
    omega_s(n) = omega_in_s;
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
plot(t, omega_s * 180 / pi)
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
