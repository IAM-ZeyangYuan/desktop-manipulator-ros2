import numpy as np
import matplotlib.pyplot as plt
import csv
import matplotlib as mpl

mpl.rcParams.update({
    "lines.linewidth": 3,
    "font.weight": "bold",
    "axes.labelweight": "bold",
    "axes.titleweight": "bold",
    "xtick.major.width": 2,
    "ytick.major.width": 2,
})



def load_csv(path):
    times, cols = [], [[], [], [], []]
    with open(path) as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames
        joint_keys = [h for h in headers if h != 'time']
        for row in reader:
            times.append(float(row['time']))
            for i, key in enumerate(joint_keys):
                cols[i].append(float(row[key]))
    return np.array(times), [np.array(c) for c in cols]


def interpolate_to_match(t_cmd, cmd, t_act, act):
    """Interpolate commanded trajectory to match actual timestamps."""
    matched_cmd = []
    for c in cmd:
        matched_cmd.append(np.interp(t_act, t_cmd, c))
    return matched_cmd


# --- load data ---
t_cmd, cmd = load_csv('commanded.csv')
t_act, act = load_csv('actual.csv')

# interpolate commanded to actual timestamps (they won't match exactly)
cmd_interp = interpolate_to_match(t_cmd, cmd, t_act, act)

joint_labels = ['Joint 1 (θ₁)', 'Joint 2 (θ₂)', 'Joint 3 (θ₃)', 'Joint 4 (d₄)']
joint_units = ['rad', 'rad', 'rad', 'm']

# --- per-joint tracking error ---
errors = []
for i in range(4):
    err = cmd_interp[i] - act[i]
    errors.append(err)

# --- compute metrics ---
print('=' * 60)
print('TRAJECTORY TRACKING PERFORMANCE REPORT')
print('=' * 60)

for i in range(4):
    err = errors[i]
    rms = np.sqrt(np.mean(err**2))
    mae = np.mean(np.abs(err))
    max_err = np.max(np.abs(err))

    # steady state error: average of last 0.5 seconds
    last_half_sec = t_act >= (t_act[-1] - 0.5)
    ss_err = np.mean(np.abs(err[last_half_sec]))

    print(f'\n{joint_labels[i]}:')
    print(f'  RMS error:          {rms:.6f} {joint_units[i]}')
    print(f'  MAE:                {mae:.6f} {joint_units[i]}')
    print(f'  Max absolute error: {max_err:.6f} {joint_units[i]}')
    print(f'  Steady-state error: {ss_err:.6f} {joint_units[i]}')


# --- Cartesian error (you need your FK function) ---
# Adjust these parameters to match your URDF (in meters)
def fk(th1, th2, th3, d4, L1=0.35, L2=0.25, L3=0.02, L4=0.10):
    c1, s1 = np.cos(th1), np.sin(th1)
    c2, s2 = np.cos(th2), np.sin(th2)
    c23, s23 = np.cos(th2 + th3), np.sin(th2 + th3)
    x = c1 * (L2 * c2 + L3 * c23 + d4 * s23)
    y = s1 * (L2 * c2 + L3 * c23 + d4 * s23)
    z = L1 + L2 * s2 + L3 * s23 - d4 * c23
    return np.array([x, y, z])


cart_errors = []
for j in range(len(t_act)):
    cmd_xyz = fk(cmd_interp[0][j], cmd_interp[1][j],
                 cmd_interp[2][j], cmd_interp[3][j])
    act_xyz = fk(act[0][j], act[1][j], act[2][j], act[3][j])
    cart_errors.append(np.linalg.norm(cmd_xyz - act_xyz))

cart_errors = np.array(cart_errors)

print(f'\nEnd-effector Cartesian error:')
print(f'  RMS:  {np.sqrt(np.mean(cart_errors**2)) * 1000:.3f} mm')
print(f'  Max:  {np.max(cart_errors) * 1000:.3f} mm')
print(f'  Mean: {np.mean(cart_errors) * 1000:.3f} mm')


# --- Plot 1: commanded vs actual per joint ---
fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
for i in range(4):
    axes[i].plot(t_act, cmd_interp[i], 'b-', label='Commanded', linewidth=1.5)
    axes[i].plot(t_act, act[i], 'r--', label='Actual', linewidth=1.5)
    axes[i].set_ylabel(f'{joint_labels[i]} ({joint_units[i]})')
    axes[i].legend(loc='upper right')
    axes[i].grid(True, alpha=0.3)
axes[-1].set_xlabel('Time (s)')
fig.suptitle('Commanded vs Actual Joint Trajectories')
plt.tight_layout()
plt.savefig('tracking_comparison.png', dpi=150)

# --- Plot 2: tracking error per joint ---
fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
for i in range(4):
    axes[i].plot(t_act, errors[i], 'k-', linewidth=1)
    axes[i].axhline(0, color='gray', linestyle='--', linewidth=0.5)
    axes[i].set_ylabel(f'{joint_labels[i]} error ({joint_units[i]})')
    axes[i].grid(True, alpha=0.3)

    # annotate RMS on the plot
    rms = np.sqrt(np.mean(errors[i]**2))
    axes[i].text(0.02, 0.95, f'RMS = {rms:.5f}',
                 transform=axes[i].transAxes, verticalalignment='top',
                 fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
axes[-1].set_xlabel('Time (s)')
fig.suptitle('Joint Tracking Error Over Time')
plt.tight_layout()
plt.savefig('tracking_error.png', dpi=150)

# --- Plot 3: Cartesian error ---
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_act, cart_errors * 1000, 'k-', linewidth=3)
ax.set_xlabel('Time (s)')
ax.set_ylabel('End-effector error (mm)')
ax.set_title('End-Effector Cartesian Tracking Error')
ax.grid(True, alpha=0.3)
rms_cart = np.sqrt(np.mean(cart_errors**2)) * 1000
ax.text(0.02, 0.95, f'RMS = {rms_cart:.3f} mm',
        transform=ax.transAxes, verticalalignment='top',
        fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
plt.tight_layout()
plt.savefig('cartesian_error.png', dpi=150)

# --- Plot 4: jerk (smoothness) ---
fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
for i in range(4):
    dt = np.diff(t_act)
    dt[dt == 0] = 1e-6  # avoid division by zero
    vel = np.diff(act[i]) / dt
    acc = np.diff(vel) / dt[:-1]
    jerk = np.diff(acc) / dt[:-2]
    axes[i].plot(t_act[:-3], jerk, 'k-', linewidth=0.5, alpha=0.7)
    axes[i].set_ylabel(f'{joint_labels[i]}')
    axes[i].grid(True, alpha=0.3)
    rms_jerk = np.sqrt(np.mean(jerk**2))
    axes[i].text(0.02, 0.95, f'RMS jerk = {rms_jerk:.3f}',
                 transform=axes[i].transAxes, verticalalignment='top',
                 fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
axes[-1].set_xlabel('Time (s)')
fig.suptitle('Joint Jerk (Smoothness Indicator)')
plt.tight_layout()
plt.savefig('jerk.png', dpi=150)

plt.show()
print('\nPlots saved.')