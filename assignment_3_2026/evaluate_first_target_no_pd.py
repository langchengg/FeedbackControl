import csv
import numpy as np

times = []
err_x = []
err_y = []
err_z = []
err_yaw = []

with open("flight_log.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        # Check if it's the first target
        if float(row['target_x']) == 2.0 and float(row['target_y']) == 2.0 and float(row['target_z']) == 2.0:
            times.append(float(row['time']))
            err_x.append(float(row['error_x']))
            err_y.append(float(row['error_y']))
            err_z.append(float(row['error_z']))
            err_yaw.append(float(row['error_yaw']))

if times:
    t_start = times[0]
    t_end = times[-1]
    duration = t_end - t_start
    if duration > 2.0:
        mid_time = t_start + duration / 2
        steady_mask = [t > mid_time for t in times]
        
        sx = [abs(x) for x, m in zip(err_x, steady_mask) if m]
        sy = [abs(y) for y, m in zip(err_y, steady_mask) if m]
        sz = [abs(z) for z, m in zip(err_z, steady_mask) if m]
        syaw = [abs(y) for y, m in zip(err_yaw, steady_mask) if m]
        
        orig_sx = [x for x, m in zip(err_x, steady_mask) if m]
        orig_sy = [y for y, m in zip(err_y, steady_mask) if m]
        orig_sz = [z for z, m in zip(err_z, steady_mask) if m]
        orig_syaw = [y for y, m in zip(err_yaw, steady_mask) if m]
        
        print(f"Evaluated Target 1 Segment (time: {mid_time:.2f}s to {t_end:.2f}s, length: {duration:.2f}s total)")
        print(f"X mean error (abs): {np.mean(sx):.6f}, std: {np.std(orig_sx):.6f}")
        print(f"Y mean error (abs): {np.mean(sy):.6f}, std: {np.std(orig_sy):.6f}")
        print(f"Z mean error (abs): {np.mean(sz):.6f}, std: {np.std(orig_sz):.6f}")
        print(f"Yaw mean error (abs): {np.mean(syaw):.6f}, std: {np.std(orig_syaw):.6f}")
    else:
        print(f"Not enough time on Target 1 (only {duration:.2f}s).")
else:
    print("Could not find the first target segment in the log.")
