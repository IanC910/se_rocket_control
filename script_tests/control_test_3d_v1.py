import math
import matplotlib.pyplot as plt
import numpy as np

def clamp(value, low, high):
    return min(max(value, low), high)

# Desired position
x_ref = np.array([100.0, 100.0, -1000.0])

# Ship properties
mass_kg = 1.0
max_thrust_N = 100.0

# World constants
earth_grav_acc  = 9.8
max_speed       = 100

# Sim constants
sim_time        = 20.0
time_step       = 0.02
num_steps       = int(sim_time / time_step)
time            = (np.arange(num_steps) + 1) * time_step

# Controller constants
gain_vel    = 0.5
gain_acc    = 2.0
gain_jerk   = 40.0
gain_omega  = 10.0
max_omega   = 30 / 60 * 2 * math.pi

# Sim variables
F_g             = np.array([0.0, 0.0, -mass_kg * earth_grav_acc])
pos             = np.zeros((num_steps, 3))
vel             = np.zeros((num_steps, 3))
acc             = np.zeros((num_steps, 3))
F_thrust_mag    = np.zeros(num_steps)
ship_frame      = np.zeros((num_steps, 3, 3))
forward              = np.zeros((num_steps, 3))
omega_s         = np.zeros((num_steps, 3))

ship_frame[0] = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
acc[0] = F_g / mass_kg
F_thrust_mag[0] = mass_kg * earth_grav_acc

for n in range(num_steps):
    p = max(n - 1, 0)
    pos0            = pos[p]
    vel0            = vel[p]
    acc0            = acc[p]
    ship_frame0     = ship_frame[p]
    F_thrust_mag0   = F_thrust_mag[p]

    # Control
    v_ref_uncapped = gain_vel * (x_ref - pos0)
    v_ref_mag = clamp(np.linalg.norm(v_ref_uncapped), 0.0, max_speed)
    v_ref = v_ref_mag * v_ref_uncapped / np.linalg.norm(v_ref_uncapped) if np.linalg.norm(v_ref_uncapped) > 0 else np.zeros(3)

    a_ref = gain_acc * (v_ref - vel0)

    j_ref = gain_jerk * (a_ref - acc0)
    F_thrust_prime_ideal = mass_kg * j_ref

    forward0 = ship_frame0[:, 0]
    F_thrust_ideal = F_thrust_mag0 * forward0 + F_thrust_prime_ideal * time_step
    F_thrust_mag_control = clamp(np.dot(F_thrust_ideal, forward0), 0.0, max_thrust_N)

    # Get thrust vector in ship frame
    F_thrust_ref_ship_frame = np.linalg.solve(ship_frame0, F_thrust_ideal)
    F_thrust_ref_mag_ship_frame = np.linalg.norm(F_thrust_ref_ship_frame)
    F_thrust_ref_dir_ship_frame = F_thrust_ref_ship_frame / F_thrust_ref_mag_ship_frame if F_thrust_ref_mag_ship_frame > 0 else np.zeros(3)

    forward_ship_frame = np.array([1.0, 0.0, 0.0])

    # Delta theta in ship frame
    forward_cross_F_thrust_dir_ship_frame = np.cross(forward_ship_frame, F_thrust_ref_dir_ship_frame)
    cross_norm = np.linalg.norm(forward_cross_F_thrust_dir_ship_frame)
    delta_theta_mag = math.asin(clamp(cross_norm, -1.0, 1.0))
    delta_theta_dir = forward_cross_F_thrust_dir_ship_frame / cross_norm if cross_norm > 0 else np.zeros(3)

    # omega control in ship frame
    omega_control_mag = clamp(gain_omega * delta_theta_mag, 0.0, max_omega)
    omega_control_dir = delta_theta_dir
    omega_control_ship_frame = omega_control_mag * omega_control_dir

    # Update sim variables
    F_net = F_thrust_mag_control * forward0 + F_g
    acc[n] = F_net / mass_kg
    vel[n] = vel0 + acc[n] * time_step
    pos[n] = pos0 + vel0 * time_step + 0.5 * acc[n] * time_step * time_step
    F_thrust_mag[n] = F_thrust_mag_control

    dtheta_ship_frame = omega_control_ship_frame * time_step
    cx, sx = math.cos(dtheta_ship_frame[0]), math.sin(dtheta_ship_frame[0])
    cy, sy = math.cos(dtheta_ship_frame[1]), math.sin(dtheta_ship_frame[1])
    cz, sz = math.cos(dtheta_ship_frame[2]), math.sin(dtheta_ship_frame[2])
    R_roll = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    R_pitch = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    R_yaw = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    ship_frame[n] = ship_frame0 @ R_roll @ R_pitch @ R_yaw
    forward[n] = ship_frame[n][:, 0]
    omega_s[n] = omega_control_ship_frame

names = ("x", "v", "forward", "omega (deg/s)", "F_thrust_mag")
values = (pos, vel, forward, np.degrees(omega_s), F_thrust_mag)
fig, axes = plt.subplots(5, 1, figsize=(8, 12), sharex=True)
for ax, value, name in zip(axes, values, names):
    ax.plot(time, value)
    ax.set_title(name)
    ax.grid(True)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()

for title, dims in (("xy path", (0, 1)), ("xz path", (0, 2)), ("yz path", (1, 2))):
    plt.figure(figsize=(7, 7))
    plt.plot(pos[:, dims[0]], pos[:, dims[1]])
    plt.xlabel("component " + str(dims[0] + 1))
    plt.ylabel("component " + str(dims[1] + 1))
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.plot(pos[:, 0], pos[:, 1], pos[:, 2])
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("path")
plt.tight_layout()
plt.show()