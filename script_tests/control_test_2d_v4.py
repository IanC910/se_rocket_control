import math
import matplotlib.pyplot as plt
import numpy as np


def clamp(value, low, high):
    return min(max(value, low), high)


def shortest_angle(distance):
    if abs(distance + 2 * math.pi) < abs(distance):
        distance += 2 * math.pi
    if abs(distance - 2 * math.pi) < abs(distance):
        distance -= 2 * math.pi
    return distance


g = 9.8
sim_time = 60.0
dt = 0.001
num_steps = int(sim_time / dt)
t = (np.arange(num_steps) + 1) * dt

x_ref = np.array([100.0, 1000.0])
K_v, K_a, K_j, K_omega = 0.5, 2.0, 40.0, 80.0
max_omega = 30 / 60 * 2 * math.pi

m = 1.0
F_g = np.array([0.0, -m * g])
x = np.zeros((2, num_steps))
v = np.zeros((2, num_steps))
a = np.zeros((2, num_steps))
F_thrust_mag = np.zeros(num_steps)
theta = np.zeros(num_steps)
omega = np.zeros(num_steps)

a[:, 0] = F_g / m
F_thrust_mag[0] = m * g
theta[0] = math.pi / 2

for n in range(num_steps):
    p = max(n - 1, 0)
    x0, v0, a0 = x[:, p], v[:, p], a[:, p]
    F_thrust0, theta0, omega0 = F_thrust_mag[p], theta[p], omega[p]
    dir0 = np.array([math.cos(theta0), math.sin(theta0)])

    v_desired = -K_v * (x0 - x_ref)
    v_desired_mag = clamp(np.linalg.norm(v_desired), 0.0, 100.0)
    if np.linalg.norm(v_desired) > 0:
        v_desired = v_desired_mag * v_desired / np.linalg.norm(v_desired)

    a_desired = -K_a * (v0 - v_desired)
    j_desired = -K_j * (a0 - a_desired)
    F_thrust_dot_desired = m * j_desired

    F_thrust_desired = F_thrust0 * dir0 + F_thrust_dot_desired * dt
    F_thrust_mag_in = clamp(np.dot(F_thrust_desired, dir0), 0.0, 100.0)
    theta_desired = math.atan2(F_thrust_desired[1], F_thrust_desired[0])
    theta_distance = shortest_angle(theta0 - theta_desired)
    omega_in = clamp(-K_omega * theta_distance, -max_omega, max_omega)

    a[:, n] = (F_thrust_mag_in * dir0 + F_g) / m
    v[:, n] = v0 + a[:, n] * dt
    x[:, n] = x0 + v0 * dt + 0.5 * a[:, n] * dt * dt
    F_thrust_mag[n] = F_thrust_mag_in
    theta[n] = theta0 + omega_in * dt
    omega[n] = omega_in

names = ("x_1", "x_2", "v_1", "v_2", "theta (deg)", "omega (deg/s)", "F_thrust_mag")
values = (x[0], x[1], v[0], v[1], np.degrees(theta), np.degrees(omega), F_thrust_mag)
fig, axes = plt.subplots(7, 1, figsize=(8, 14), sharex=True)
for ax, value, name in zip(axes, values, names):
    ax.plot(t, value)
    ax.set_title(name)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()

plt.figure(figsize=(7, 7))
plt.plot(x[0], x[1])
plt.xlabel("x_1")
plt.ylabel("x_2")
plt.title("path")
plt.axis("equal")
plt.grid(True)
plt.show()
