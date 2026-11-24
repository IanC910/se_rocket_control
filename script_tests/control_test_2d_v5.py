import math
import matplotlib.pyplot as plt
import numpy as np


def clamp(value, low, high):
    return min(max(value, low), high)


g = 9.8
sim_time = 60.0
dt = 0.02
num_steps = int(sim_time / dt)
t = (np.arange(num_steps) + 1) * dt

x_ref = np.array([100.0, 1000.0])
K_v, K_a, K_j, K_omega = 0.5, 2.0, 40.0, 10.0
max_omega = 30 / 60 * 2 * math.pi

m = 1.0
F_g = np.array([0.0, -m * g])
x = np.zeros((2, num_steps))
v = np.zeros((2, num_steps))
a = np.zeros((2, num_steps))
F_thrust_mag = np.zeros(num_steps)
B = np.zeros((2, 2, num_steps))
theta = np.zeros(num_steps)
omega_s = np.zeros(num_steps)

B[:, :, 0] = np.array([[0.0, -1.0], [1.0, 0.0]])
a[:, 0] = F_g / m
F_thrust_mag[0] = m * g
theta[0] = math.atan2(B[1, 0, 0], B[0, 0, 0])

for n in range(num_steps):
    p = max(n - 1, 0)
    x0, v0, a0 = x[:, p], v[:, p], a[:, p]
    B0 = B[:, :, p]
    dir0 = B0[:, 0]
    F_thrust0 = F_thrust_mag[p]

    v_ref_uncapped = -K_v * (x0 - x_ref)
    norm_uncapped = np.linalg.norm(v_ref_uncapped)
    v_ref_mag = clamp(norm_uncapped, 0.0, 100.0)
    v_ref = v_ref_mag * v_ref_uncapped / norm_uncapped if norm_uncapped > 0 else np.zeros(2)

    a_ref = -K_a * (v0 - v_ref)
    j_ref = -K_j * (a0 - a_ref)
    F_thrust_dot_ref = m * j_ref

    F_thrust_ref = F_thrust0 * dir0 + F_thrust_dot_ref * dt
    F_thrust_mag_in = clamp(np.dot(F_thrust_ref, dir0), 0.0, 100.0)

    F_thrust_ref_s = np.linalg.solve(B0, F_thrust_ref)
    norm_ref_s = np.linalg.norm(F_thrust_ref_s)
    F_thrust_ref_dir_s = F_thrust_ref_s / norm_ref_s if norm_ref_s > 0 else np.zeros(2)
    b_1_s = np.array([1.0, 0.0])

    cross_z = b_1_s[0] * F_thrust_ref_dir_s[1] - b_1_s[1] * F_thrust_ref_dir_s[0]
    delta_theta_s = math.asin(clamp(cross_z, -1.0, 1.0))
    omega_in_s = clamp(K_omega * delta_theta_s, -max_omega, max_omega)

    b1 = B0[:, 0]
    F_net = F_thrust_mag_in * b1 + F_g
    a[:, n] = F_net / m
    v[:, n] = v0 + a[:, n] * dt
    x[:, n] = x0 + v0 * dt + 0.5 * a[:, n] * dt * dt
    F_thrust_mag[n] = F_thrust_mag_in

    dtheta = omega_in_s * dt
    rotation = np.array([[math.cos(dtheta), -math.sin(dtheta)], [math.sin(dtheta), math.cos(dtheta)]])
    B[:, :, n] = B0 @ rotation
    theta[n] = theta[p] + dtheta
    omega_s[n] = omega_in_s

names = ("x_1", "x_2", "v_1", "v_2", "theta (deg)", "omega (deg/s)", "F_thrust_mag")
values = (x[0], x[1], v[0], v[1], np.degrees(theta), np.degrees(omega_s), F_thrust_mag)
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
