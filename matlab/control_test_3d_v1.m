
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
sim_time    = 20;
dt          = 0.02;

num_steps   = sim_time / dt;
steps = 1 : num_steps;
t = steps * dt;

% Desired Conditions
x_ref = [100; 100; -1000];

% Controller values
K_v         = 0.5;
K_a         = 2;
K_j         = 40;
K_omega     = 10;

max_omega = 30 / 60 * 2 * pi;
max_speed = 100;
max_thrust = 100;

% Initial Conditions
Z1 = zeros(1, num_steps);
Z2 = zeros(2, num_steps);
Z3 = zeros(3, num_steps);

m = 1;
F_g = [0; 0; -m * g];

x               = Z3;
v               = Z3;
a               = Z3;
F_thrust_mag    = Z1;
B               = zeros(3, 3, num_steps); B(:, :, 1) = [0 0 1; 1 0 0; 0 1 0];
b1              = Z3;
omega_s         = Z3;

a(:, 1) = F_g / m;
F_thrust_mag(1) = m * g;

% Start of sim

for n = steps
    p           = max([n - 1, 1]);
    x0          = x(:, p);
    v0          = v(:, p);
    a0          = a(:, p);
    F_thrust0   = F_thrust_mag(p);
    B0          = B(:, :, p);

    % Control
    v_ref_uncapped = -K_v * (x0 - x_ref);
    v_ref_mag = min(norm(v_ref_uncapped), max_speed);
    v_ref = v_ref_mag * v_ref_uncapped / norm(v_ref_uncapped);

    a_ref = -K_a * (v0 - v_ref);

    j_ref = -K_j * (a0 - a_ref);
    F_thrust_dot_ref = m * j_ref;

    F_thrust_ref = F_thrust0 * B0(:, 1) + F_thrust_dot_ref * dt;
    F_thrust_mag_in_uncapped = dot(F_thrust_ref, B0(:, 1));
    F_thrust_mag_in = min(max(F_thrust_mag_in_uncapped, 0), max_thrust);

    % Get thrust vector in ship coords
    F_thrust_ref_s = B0 \ F_thrust_ref;
    F_thrust_ref_dir_s = F_thrust_ref_s / norm(F_thrust_ref_s);
    b_1_s = [1; 0; 0]; % Forward direction in ship coords

    % Delta theta in ship coords
    delta_theta_temp_s = cross(b_1_s, F_thrust_ref_dir_s);
    delta_theta_mag = asin(norm(delta_theta_temp_s));
    delta_theta_s = delta_theta_mag * delta_theta_temp_s / norm(delta_theta_temp_s);

    omega_in_uncapped_s = K_omega * delta_theta_s;
    omega_in_mag = min(norm(omega_in_uncapped_s), max_omega);
    omega_in_s = omega_in_mag * omega_in_uncapped_s / norm(omega_in_uncapped_s);

    % Update state
    F_net   = F_thrust_mag_in * B0(:, 1) + F_g;
    a(:, n) = F_net / m;
    v(:, n) = v0 + a(:, n) * dt;
    x(:, n) = x0 + v0 * dt + 0.5 * a(:, n) * dt * dt;
    F_thrust_mag(n) = F_thrust_mag_in;

    dtheta_s = omega_in_s * dt;
    R_roll = [1 0 0; 0 cos(dtheta_s(1)) -sin(dtheta_s(1)); 0 sin(dtheta_s(1)) cos(dtheta_s(1))];
    R_pitch = [cos(dtheta_s(2)) 0 sin(dtheta_s(2)); 0 1 0; -sin(dtheta_s(2)) 0 cos(dtheta_s(2))];
    R_yaw = [cos(dtheta_s(3)) -sin(dtheta_s(3)) 0; sin(dtheta_s(3)) cos(dtheta_s(3)) 0; 0 0 1];
    B(:, :, n) = B0 * R_roll * R_pitch * R_yaw;
    b1(:, n) = B(:, 1, n);
    omega_s(:, n) = omega_in_s;
end

x_limits = [min(min(min(x))), max(max(max(x)))];

figure
tiledlayout(5, 1)

nexttile
plot(t, x)
title('x')

nexttile
plot(t, v)
title('v')

nexttile
plot(t, b1)
title('b1')

nexttile
plot(t, omega_s * 180 / pi)
title('omega in deg/s')

nexttile
plot(t, F_thrust_mag)
title('F\_thrust_mag')
hold off


x_limits2 = [x_limits, x_limits];
x_limits3 = [x_limits, x_limits, x_limits];

figure
title('xy path')
hold on
for s = 0 : sim_time - 1
    plot(x(1, s/dt + 1: (s + 1)/dt), x(2, s/dt + 1: (s + 1)/dt))
    axis(x_limits2)
end
hold off


figure
title('xz path')
hold on
for s = 0 : sim_time - 1
    plot(x(1, s/dt + 1: (s + 1)/dt), x(3, s/dt + 1: (s + 1)/dt))
    axis(x_limits2)
end
hold off


figure
title('yz path')
hold on
for s = 0 : sim_time - 1
    plot(x(2, s/dt + 1: (s + 1)/dt), x(3, s/dt + 1: (s + 1)/dt))
    axis(x_limits2)
end
hold off

figure
title('path')
plot3(x(1, :), x(2, :), x(3, :))
axis(x_limits3);
