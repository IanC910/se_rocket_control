import math
import matplotlib.pyplot as plt
import numpy as np


def clamp(value, low, high):
    return min(max(value, low), high)


g = 9.8
sim_time = 20.0
dt = 0.02
num_steps = int(sim_time / dt)
t = (np.arange(num_steps) + 1) * dt

x_ref = np.array([100.0, 100.0, -1000.0])
K_v, K_a, K_j, K_omega = 0.5, 2.0, 40.0, 10.0
max_omega = 30 / 60 * 2 * math.pi
max_speed = 100.0
max_thrust = 100.0

m = 1.0
F_g = np.array([0.0, 0.0, -m * g])
x = np.zeros((3, num_steps))
v = np.zeros((3, num_steps))
a = np.zeros((3, num_steps))
F_thrust_mag = np.zeros(num_steps)
B = np.zeros((3, 3, num_steps))
b1 = np.zeros((3, num_steps))
omega_s = np.zeros((3, num_steps))

B[:, :, 0] = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
a[:, 0] = F_g / m
F_thrust_mag[0] = m * g

for n in range(num_steps):
    p = max(n - 1, 0)
    x0, v0, a0 = x[:, p], v[:, p], a[:, p]
    B0 = B[:, :, p]
    F_thrust0 = F_thrust_mag[p]

    v_ref_uncapped = -K_v * (x0 - x_ref)
    norm_uncapped = np.linalg.norm(v_ref_uncapped)
    v_ref_mag = clamp(norm_uncapped, 0.0, max_speed)
    v_ref = v_ref_mag * v_ref_uncapped / norm_uncapped if norm_uncapped > 0 else np.zeros(3)

    a_ref = -K_a * (v0 - v_ref)
    j_ref = -K_j * (a0 - a_ref)
    F_thrust_dot_ref = m * j_ref

    F_thrust_ref = F_thrust0 * B0[:, 0] + F_thrust_dot_ref * dt
    F_thrust_mag_in = clamp(np.dot(F_thrust_ref, B0[:, 0]), 0.0, max_thrust)

    F_thrust_ref_s = np.linalg.solve(B0, F_thrust_ref)
    norm_ref_s = np.linalg.norm(F_thrust_ref_s)
    if norm_ref_s == 0:
        F_thrust_ref_dir_s = np.zeros(3)
    else:
        F_thrust_ref_dir_s = F_thrust_ref_s / norm_ref_s

    b_1_s = np.array([1.0, 0.0, 0.0])
    cross_delta = np.cross(b_1_s, F_thrust_ref_dir_s)
    cross_norm = np.linalg.norm(cross_delta)
    delta_theta_s = math.asin(clamp(cross_norm, -1.0, 1.0)) * cross_delta / cross_norm if cross_norm > 0 else np.zeros(3)

    omega_in_uncapped_s = K_omega * delta_theta_s
    omega_norm = np.linalg.norm(omega_in_uncapped_s)
    omega_in_s = clamp(omega_norm, 0.0, max_omega) * omega_in_uncapped_s / omega_norm if omega_norm > 0 else np.zeros(3)

    F_net = F_thrust_mag_in * B0[:, 0] + F_g
    a[:, n] = F_net / m
    v[:, n] = v0 + a[:, n] * dt
    x[:, n] = x0 + v0 * dt + 0.5 * a[:, n] * dt * dt
    F_thrust_mag[n] = F_thrust_mag_in

    dtheta_s = omega_in_s * dt
    cx, sx = math.cos(dtheta_s[0]), math.sin(dtheta_s[0])
    cy, sy = math.cos(dtheta_s[1]), math.sin(dtheta_s[1])
    cz, sz = math.cos(dtheta_s[2]), math.sin(dtheta_s[2])
    R_roll = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    R_pitch = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    R_yaw = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    B[:, :, n] = B0 @ R_roll @ R_pitch @ R_yaw
    b1[:, n] = B[:, 0, n]
    omega_s[:, n] = omega_in_s

names = ("x", "v", "b1", "omega (deg/s)", "F_thrust_mag")
values = (x, v, b1, np.degrees(omega_s), F_thrust_mag)
fig, axes = plt.subplots(5, 1, figsize=(8, 12), sharex=True)
for ax, value, name in zip(axes, values, names):
    ax.plot(t, value.T)
    ax.set_title(name)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()

for title, dims in (("xy path", (0, 1)), ("xz path", (0, 2)), ("yz path", (1, 2))):
    plt.figure(figsize=(7, 7))
    plt.plot(x[dims[0]], x[dims[1]])
    plt.xlabel("component " + str(dims[0] + 1))
    plt.ylabel("component " + str(dims[1] + 1))
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.plot(x[0], x[1], x[2])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("path")
plt.tight_layout()
plt.show()
