import math
import matplotlib.pyplot as plt
import numpy as np


g = 9.8
sim_time = 10.0
dt = 0.001
num_steps = int(sim_time / dt)
t = (np.arange(num_steps) + 1) * dt

x_ref = 10.0
K_v = 1.0
K_a = 2.5
K_omega = 10.0
max_theta = math.pi / 4
max_omega = 30 / 60 * 2 * math.pi

m = 1.0
x = np.zeros(num_steps)
v_x = np.zeros(num_steps)
theta = np.zeros(num_steps)
omega = np.zeros(num_steps)
F_g = m * g

for n in range(num_steps):
    p = max(n - 1, 0)
    x0 = x[p]
    v_x0 = v_x[p]
    theta0 = theta[p]

    v_x_in = -K_v * (x0 - x_ref)
    a_x_in = -K_a * (v_x0 - v_x_in)
    theta_in = math.atan(-a_x_in / g)
    theta_in = min(max(theta_in, -max_theta), max_theta)

    omega_in = -K_omega * (theta0 - theta_in)
    omega_in = min(max(omega_in, -max_omega), max_omega)

    F_thrust = F_g / math.cos(theta0)
    F_thrust_x = F_thrust * -math.sin(theta0)

    a_x = F_thrust_x / m
    v_x[n] = v_x0 + a_x * dt
    x[n] = x0 + v_x0 * dt + 0.5 * a_x * dt * dt
    theta[n] = theta0 + omega_in * dt
    omega[n] = omega_in

fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 10))
for ax, values, title in zip(
    axes,
    (x, v_x, np.degrees(theta), np.degrees(omega)),
    ("x", "v", "theta (deg)", "omega (deg/s)"),
):
    ax.plot(t, values)
    ax.set_title(title)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()
plt.show()
