import math
import matplotlib.pyplot as plt
import numpy as np


g = 9.8
sim_time = 10.0
dt = 0.001
num_steps = int(sim_time / dt)
t = (np.arange(num_steps) + 1) * dt

x_ref = 10.0
y_ref = 10.0
K_v = 1.0
K_a = 2.5
K_omega = 10.0
max_theta = math.pi / 4
max_omega = 30 / 60 * 2 * math.pi

m = 1.0
x = np.zeros(num_steps)
y = np.zeros(num_steps)
v_x = np.zeros(num_steps)
v_y = np.zeros(num_steps)
theta = np.zeros(num_steps)
omega = np.zeros(num_steps)
F_g = m * g

for n in range(num_steps):
    p = max(n - 1, 0)
    x0, y0 = x[p], y[p]
    v_x0, v_y0 = v_x[p], v_y[p]
    theta0 = theta[p]

    v_x_in = -K_v * (x0 - x_ref)
    v_y_in = -K_v * (y0 - y_ref)
    a_x_in = -K_a * (v_x0 - v_x_in)
    a_y_in = -K_a * (v_y0 - v_y_in)

    theta_in = min(max(math.atan(-a_x_in / g), -max_theta), max_theta)
    omega_in = -K_omega * (theta0 - theta_in)
    omega_in = min(max(omega_in, -max_omega), max_omega)

    F_thrust_y = m * a_y_in + F_g
    F_thrust = F_thrust_y / math.cos(theta0)
    a_x = F_thrust * -math.sin(theta0) / m
    a_y = (F_thrust * math.cos(theta0) - F_g) / m

    v_x[n] = v_x0 + a_x * dt
    v_y[n] = v_y0 + a_y * dt
    x[n] = x0 + v_x0 * dt + 0.5 * a_x * dt * dt
    y[n] = y0 + v_y0 * dt + 0.5 * a_y * dt * dt
    theta[n] = theta0 + omega_in * dt
    omega[n] = omega_in

names = ("x", "y", "v_x", "v_y", "theta (deg)", "omega (deg/s)")
values = (x, y, v_x, v_y, np.degrees(theta), np.degrees(omega))
fig, axes = plt.subplots(len(values), 1, sharex=True, figsize=(8, 12))
for ax, value, name in zip(axes, values, names):
    ax.plot(t, value)
    ax.set_title(name)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()
plt.show()
