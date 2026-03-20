#!/usr/bin/env python3
"""
plot_data.py  –  Post-flight analysis & video-ready charts for Layer 1
======================================================================
Run AFTER a simulation session.  Reads 'flight_log.csv' produced by the
DataLogger inside controller.py and generates publication-quality figures.

Usage:
    python plot_data.py                   # uses default flight_log.csv
    python plot_data.py my_log.csv        # specify a different log file

Output figures (saved as PNG in current directory):
    1. 3d_trajectory.png      – 3D flight path with start/target markers
    2. position_error.png     – X, Y, Z position error vs time
    3. yaw_tracking.png       – Yaw actual vs target & yaw error vs time
    4. velocity_commands.png  – Commanded vx, vy, vz, yaw_rate vs time
    5. settling_analysis.png  – Combined RMS position error + settling band
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # use a GUI backend so plt.show() works
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ── Load data ───────────────────────────────────────────────────────────────
csv_file = sys.argv[1] if len(sys.argv) > 1 else "flight_log.csv"
try:
    data = np.genfromtxt(csv_file, delimiter=",", names=True)
except FileNotFoundError:
    print(f"ERROR: '{csv_file}' not found.  Run the simulator first to generate it.")
    sys.exit(1)

t        = data["time"]
px, py, pz = data["pos_x"], data["pos_y"], data["pos_z"]
yaw      = data["yaw"]
tx, ty, tz = data["target_x"], data["target_y"], data["target_z"]
tyaw     = data["target_yaw"]
ex, ey, ez = data["error_x"], data["error_y"], data["error_z"]
eyaw     = data["error_yaw"]
cvx, cvy, cvz = data["cmd_vx"], data["cmd_vy"], data["cmd_vz"]
cyaw     = data["cmd_yaw_rate"]

# ─────────────────────────────────────────────────────────────────────────────
# Figure 1 : 3D Trajectory
# ─────────────────────────────────────────────────────────────────────────────
fig1 = plt.figure(figsize=(8, 6))
ax1 = fig1.add_subplot(111, projection='3d')
ax1.plot(px, py, pz, 'b-', linewidth=0.8, label='Flight Path')
ax1.scatter(px[0], py[0], pz[0], c='green', s=80, marker='o', label='Start')
ax1.scatter(tx[-1], ty[-1], tz[-1], c='red', s=120, marker='*', label='Target')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Flight Trajectory')
ax1.legend()
fig1.tight_layout()
fig1.savefig("3d_trajectory.png", dpi=150)
print("Saved 3d_trajectory.png")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 2 : Position Error vs Time (X, Y, Z separately)
# ─────────────────────────────────────────────────────────────────────────────
fig2, axes2 = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
labels = ['X Error (m)', 'Y Error (m)', 'Z Error (m)']
errors = [ex, ey, ez]
colours = ['#e74c3c', '#2ecc71', '#3498db']

for i, ax in enumerate(axes2):
    ax.plot(t, errors[i], color=colours[i], linewidth=0.8)
    ax.axhline(0, color='grey', linestyle='--', linewidth=0.5)
    ax.axhline( 0.01, color='orange', linestyle=':', linewidth=0.8, label='±0.01 m threshold')
    ax.axhline(-0.01, color='orange', linestyle=':', linewidth=0.8)
    ax.set_ylabel(labels[i])
    if i == 0:
        ax.legend(loc='upper right', fontsize=8)

axes2[-1].set_xlabel('Time (s)')
fig2.suptitle('Position Error vs Time', fontsize=13)
fig2.tight_layout()
fig2.savefig("position_error.png", dpi=150)
print("Saved position_error.png")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 3 : Yaw Tracking
# ─────────────────────────────────────────────────────────────────────────────
fig3, (ax3a, ax3b) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
ax3a.plot(t, np.degrees(yaw), 'b-', linewidth=0.8, label='Actual Yaw')
ax3a.plot(t, np.degrees(tyaw), 'r--', linewidth=0.8, label='Target Yaw')
ax3a.set_ylabel('Yaw (°)')
ax3a.legend(fontsize=8)
ax3a.set_title('Yaw Tracking')

ax3b.plot(t, np.degrees(eyaw), 'm-', linewidth=0.8)
ax3b.axhline(0, color='grey', linestyle='--', linewidth=0.5)
ax3b.axhline( np.degrees(0.01), color='orange', linestyle=':', linewidth=0.8, label='±0.01 rad threshold')
ax3b.axhline(-np.degrees(0.01), color='orange', linestyle=':', linewidth=0.8)
ax3b.set_ylabel('Yaw Error (°)')
ax3b.set_xlabel('Time (s)')
ax3b.legend(fontsize=8)
fig3.tight_layout()
fig3.savefig("yaw_tracking.png", dpi=150)
print("Saved yaw_tracking.png")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 4 : Velocity Commands
# ─────────────────────────────────────────────────────────────────────────────
fig4, axes4 = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
cmd_data   = [cvx, cvy, cvz, cyaw]
cmd_labels = ['Vx cmd (m/s)', 'Vy cmd (m/s)', 'Vz cmd (m/s)', 'Yaw rate cmd (rad/s)']
cmd_colors = ['#e74c3c', '#2ecc71', '#3498db', '#9b59b6']

for i, ax in enumerate(axes4):
    ax.plot(t, cmd_data[i], color=cmd_colors[i], linewidth=0.8)
    ax.axhline(0, color='grey', linestyle='--', linewidth=0.5)
    ax.set_ylabel(cmd_labels[i], fontsize=9)

axes4[-1].set_xlabel('Time (s)')
fig4.suptitle('Controller Velocity Commands', fontsize=13)
fig4.tight_layout()
fig4.savefig("velocity_commands.png", dpi=150)
print("Saved velocity_commands.png")

# ─────────────────────────────────────────────────────────────────────────────
# Figure 5 : Settling Analysis – RMS position error over time
# ─────────────────────────────────────────────────────────────────────────────
rms_error = np.sqrt(ex**2 + ey**2 + ez**2)

fig5, ax5 = plt.subplots(figsize=(10, 4))
ax5.plot(t, rms_error, 'k-', linewidth=0.8, label='RMS Position Error')
ax5.axhline(0.01, color='red', linestyle='--', linewidth=1.0, label='0.01 m threshold')
ax5.fill_between(t, 0, 0.01, alpha=0.1, color='green', label='Acceptable region')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('RMS Error (m)')
ax5.set_title('Settling Analysis – RMS Position Error')
ax5.legend(fontsize=9)
ax5.set_ylim(bottom=0)
fig5.tight_layout()
fig5.savefig("settling_analysis.png", dpi=150)
print("Saved settling_analysis.png")

# ─────────────────────────────────────────────────────────────────────────────
# Summary Statistics (printed to terminal)
# ─────────────────────────────────────────────────────────────────────────────
# Use last 50 % of data (assumed "settled") for steady-state metrics
half = len(t) // 2
print("\n" + "="*60)
print("  STEADY-STATE PERFORMANCE (last 50% of data)")
print("="*60)
print(f"  Position Error Mean :  X={np.mean(np.abs(ex[half:])):.5f} m  "
      f"Y={np.mean(np.abs(ey[half:])):.5f} m  Z={np.mean(np.abs(ez[half:])):.5f} m")
print(f"  Position Error Std  :  X={np.std(ex[half:]):.5f} m  "
      f"Y={np.std(ey[half:]):.5f} m  Z={np.std(ez[half:]):.5f} m")
print(f"  RMS Error Mean      :  {np.mean(rms_error[half:]):.5f} m")
print(f"  Yaw Error Mean      :  {np.mean(np.abs(eyaw[half:])):.5f} rad")
print(f"  Yaw Error Std       :  {np.std(eyaw[half:]):.5f} rad")
print("="*60)

# Determine settling time (first time RMS error stays below 0.01 for 1 second)
settled = False
settle_time = None
for i in range(len(rms_error)):
    if rms_error[i] < 0.01:
        if not settled:
            candidate = t[i]
            settled = True
        if t[i] - candidate >= 1.0:
            settle_time = candidate
            break
    else:
        settled = False

if settle_time is not None:
    print(f"  Settling Time (1s within 0.01m): {settle_time:.2f} s")
else:
    print("  Settling Time: NOT achieved within recording")
print("="*60)

plt.show()
