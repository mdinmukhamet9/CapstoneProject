# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np

# # Load the CSV data
# df = pd.read_csv("robot_log.csv")

# # Calculate time since first command
# df["time_elapsed"] = df["current_time"] - df["current_time"].iloc[0]

# # Convert to numpy arrays explicitly to avoid pandas/matplotlib issues
# time_values = df["time_elapsed"].to_numpy()
# velocity_values = df["current_linear"].to_numpy()
# desired_velocity = df["Vdesired_linear"].iloc[0]

# # Create the plot
# plt.figure(figsize=(12, 6))

# # Plot linear velocity
# plt.plot(time_values, velocity_values, 
#         color='blue', linewidth=2, label="Velocity (m/s)")

# movement_start_mask = velocity_values > 0.01*desired_velocity
# if np.any(movement_start_mask):
#     first_movement_idx = np.argmax(movement_start_mask)
#     plt.scatter(time_values[first_movement_idx], velocity_values[first_movement_idx],
#                color='green', s=100, zorder=5, label="Movement start")

# # Find and mark goal reached (first time within 2% of target)
# goal_tolerance = 0.01 * desired_velocity
# goal_reached_mask = np.abs(velocity_values - desired_velocity) < goal_tolerance
# if np.any(goal_reached_mask):
#     goal_reached_idx = np.argmax(goal_reached_mask)
#     plt.scatter(time_values[goal_reached_idx], velocity_values[goal_reached_idx],
#                color='red', s=100, zorder=5, label="Goal reached")

# # Add horizontal line for desired velocity
# plt.axhline(y=desired_velocity, color='purple', linestyle=':', 
#             label=f'Target ({desired_velocity} m/s)')

# # Formatting
# plt.title("Robot Velocity Over Time", fontsize=14)
# plt.xlabel("Time (seconds)", fontsize=12)
# plt.ylabel("Velocity (m/s)", fontsize=12)
# plt.grid(True, alpha=0.3)
# plt.legend(fontsize=10)

# # Save and show
# plt.tight_layout()
# plt.savefig("velocity_plot.png", dpi=300, bbox_inches='tight')
# plt.show()


import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV data
df = pd.read_csv("robot_log.csv")

# Calculate time since first command
df["time_elapsed"] = df["current_time"] - df["current_time"].iloc[0]

# Convert to numpy arrays
time_values = df["time_elapsed"].to_numpy()
velocity_values = df["current_linear"].to_numpy()
desired_velocity = df["Vdesired_linear"].iloc[0]

# Create the plot
plt.figure(figsize=(14, 7))

# Plot linear velocity
plt.plot(time_values, velocity_values, 
        color='blue', linewidth=2.5, label="Velocity (m/s)")

# Add reference grid lines
plt.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
for vel in np.linspace(0, desired_velocity*1.1, 6):
    plt.axhline(y=vel, color='gray', linestyle=':', alpha=0.3)

# Detect and mark movement start
movement_threshold = 0.01 * desired_velocity
movement_start_mask = velocity_values > movement_threshold
if np.any(movement_start_mask):
    first_movement_idx = np.argmax(movement_start_mask)
    t1 = time_values[first_movement_idx]
    v1 = velocity_values[first_movement_idx]
    
    plt.scatter(t1, v1, color='green', s=120, zorder=5, label="Movement start")
    plt.axvline(x=t1, color='green', linestyle='--', alpha=0.5)
    
    # Annotation with exact values
    plt.annotate(f'Start: {t1:.3f}s\nVel: {v1:.3f} m/s', 
                xy=(t1, v1), xytext=(10, 20),
                textcoords='offset points', ha='left', va='bottom',
                bbox=dict(boxstyle='round,pad=0.5', fc='white', alpha=0.8),
                arrowprops=dict(arrowstyle='->'))

# Detect and mark goal reached
goal_tolerance = 0.02 * desired_velocity
goal_reached_mask = np.abs(velocity_values - desired_velocity) < goal_tolerance
if np.any(goal_reached_mask):
    goal_reached_idx = np.argmax(goal_reached_mask)
    t2 = time_values[goal_reached_idx]
    v2 = velocity_values[goal_reached_idx]
    
    plt.scatter(t2, v2, color='red', s=120, zorder=5, label="Goal reached")
    plt.axvline(x=t2, color='red', linestyle='--', alpha=0.5)
    
    # Annotation with exact values
    plt.annotate(f'Goal: {t2:.3f}s\nVel: {v2:.3f} m/s', 
                xy=(t2, v2), xytext=(-10, -30),
                textcoords='offset points', ha='right', va='top',
                bbox=dict(boxstyle='round,pad=0.5', fc='white', alpha=0.8),
                arrowprops=dict(arrowstyle='->'))

# Add horizontal line for desired velocity
plt.axhline(y=desired_velocity, color='purple', linestyle='-.', 
            linewidth=1.5, alpha=0.7, label=f'Target ({desired_velocity} m/s)')

# Calculate and display performance metrics
if np.any(movement_start_mask) and np.any(goal_reached_mask):
    latency = t1
    rise_time = t2 - t1
    plt.figtext(0.15, 0.85, f'Latency: {latency:.3f}s', fontsize=10, 
               bbox=dict(facecolor='white', alpha=0.5))
    plt.figtext(0.15, 0.82, f'Rise Time: {rise_time:.3f}s', fontsize=10,
               bbox=dict(facecolor='white', alpha=0.5))
# summary = {
#     "Command Sent Time (t0)": df["t0_command_sent"].iloc[0],
#     "Movement Start Time (t1)": df["t1_movement_start"].max(),
#     "Goal Reach Time (t2)": df["t2_goal_reached"].max(),
#     "Latency (t1 - t0)": latency,
#     "Rise Time (t2 - t1)": rise_time,
# }
# print("\nPerformance Summary:")
# for k, v in summary.items():
#     print(f"{k}: {v:.3f} seconds")

# Formatting
plt.title("Robot Velocity Profile Analysis", fontsize=16, pad=20)
plt.xlabel("Time (seconds)", fontsize=12)
plt.ylabel("Velocity (m/s)", fontsize=12)
plt.grid(True, which='both', alpha=0.3)
plt.legend(loc='upper right', fontsize=10)

# Adjust plot limits
plt.xlim(left=0)
plt.ylim(bottom=min(0, velocity_values.min()*1.1), 
        top=max(desired_velocity*1.2, velocity_values.max()*1.1))

# Save and show
plt.tight_layout()
plt.savefig("velocity_analysis.png", dpi=300, bbox_inches='tight')
plt.show()