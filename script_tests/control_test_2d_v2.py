import math
import matplotlib.pyplot as plt
import numpy as np


g = 9.8
sim_time = 20.0
dt = 0.001
num_steps = int(sim_time / dt)
t = (np.arange(num_steps) + 1) * dt

x_ref = 10.0
y_ref = -1000.0
K_v = 0.8
K_a = 10.0
K_omega = 100.0
K_j = 100.0
max_theta = math.pi / 4
max_omega = 30 / 60 * 2 * math.pi

m = 10.0
x = np.zeros(num_steps)
y = np.zeros(num_steps)
v_x = np.zeros(num_steps)
v_y = np.zeros(num_steps)
a_y = np.full(num_steps, -g)
theta = np.zeros(num_steps)
omega = np.zeros(num_steps)
F_th = np.zeros(num_steps)
F_g = m * g

for n in range(num_steps):
    p = max(n - 1, 0)
    x0, y0 = x[p], y[p]
    v_x0, v_y0 = v_x[p], v_y[p]
    theta0, F_th0 = theta[p], F_th[p]

    v_x_in = -K_v * (x0 - x_ref)
    v_y_in = -K_v * (y0 - y_ref)
    a_x_in = -K_a * (v_x0 - v_x_in)
    a_y_in = -K_a * (v_y0 - v_y_in)

    theta_in = min(max(math.atan(-a_x_in / g), -max_theta), max_theta)
    omega_in = -K_omega * (theta0 - theta_in)
    omega_in = min(max(omega_in, -max_omega), max_omega)

    F_th_y_dot = -K_j * m * (a_y[p] - a_y_in)
    F_th_dot = F_th_y_dot / math.cos(theta0)
    F_th_in = max(F_th0 + F_th_dot * dt, 0)
    F_th[n] = F_th_in

    a_x = F_th_in * -math.sin(theta0) / m
    a_y[n] = (F_th_in * math.cos(theta0) - F_g) / m
    v_x[n] = v_x0 + a_x * dt
    v_y[n] = v_y0 + a_y[n] * dt
    x[n] = x0 + v_x0 * dt + 0.5 * a_x * dt * dt
    y[n] = y0 + v_y0 * dt + 0.5 * a_y[n] * dt * dt
    theta[n] = theta0 + omega_in * dt
    omega[n] = omega_in

names = ("x", "y", "path", "v_x", "v_y", "theta (deg)", "omega (deg/s)", "F_th")
fig, axes = plt.subplots(7, 1, figsize=(8, 14), sharex=True)
for ax, value, name in zip(axes, (x, y, v_x, v_y, np.degrees(theta), np.degrees(omega), F_th), names[0:2] + names[3:]):
    ax.plot(t, value)
    ax.set_title(name)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()

plt.figure(figsize=(7, 7))
plt.plot(x, y)
plt.xlabel("x")
plt.ylabel("y")
plt.title("path")
plt.axis("equal")
plt.grid(True)
plt.show()
