import pandas as pd
import numpy as np

# Load the data
df = pd.read_csv("flight_log.csv")

# Find the segments where target is constant
# The first target is 2.0, 2.0, 2.0, 0.0
target_mask = (df['target_x'] == 2.0) & (df['target_y'] == 2.0) & (df['target_z'] == 2.0) & (df['target_yaw'] == 0.0)

# Extract only the first contiguous block (the first flight)
first_target_df = df[target_mask]
if len(first_target_df) > 0:
    # Get the time span
    t_start = first_target_df['time'].iloc[0]
    t_end = first_target_df['time'].iloc[-1]
    
    # We take the "last 50%" of this specific segment to evaluate steady-state
    duration = t_end - t_start
    if duration > 4.0:
        mid_time = t_start + duration / 2
        steady_df = first_target_df[first_target_df['time'] > mid_time]
        
        print(f"Evaluated Target 1 Segment (time: {mid_time:.2f}s to {t_end:.2f}s)")
        print(f"X mean error: {steady_df['error_x'].abs().mean():.6f}, std: {steady_df['error_x'].std():.6f}")
        print(f"Y mean error: {steady_df['error_y'].abs().mean():.6f}, std: {steady_df['error_y'].std():.6f}")
        print(f"Z mean error: {steady_df['error_z'].abs().mean():.6f}, std: {steady_df['error_z'].std():.6f}")
        print(f"Yaw mean error: {steady_df['error_yaw'].abs().mean():.6f}, std: {steady_df['error_yaw'].std():.6f}")
    else:
        print(f"Not enough time on Target 1 (only {duration:.2f}s). Need more than 4s to evaluate steady state properly.")
else:
    print("Could not find the first target segment in the log.")
