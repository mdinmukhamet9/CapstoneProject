import pandas as pd
import matplotlib.pyplot as plt

# Load the data
df = pd.read_csv('mpc_goal_data.csv')

# Create subplots
fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

df['linear_vel_error'] = df['linear_vel'] - df['actual_linear_vel']
df['angular_vel_error'] = df['angular_vel'] - df['actual_angular_vel']

axs[0].plot(df['linear_vel'], color='blue', linewidth=2.0, label='MPC Linear Velocity')
axs[0].plot(df['actual_linear_vel'], color='red', linestyle='--', linewidth=2, label='Actual Linear Velocity')
axs[0].set_ylabel('Linear Velocity (m/s)')
axs[1].set_xlabel('Time (s)')
axs[0].set_title('MPC vs. Actual Linear Velocity')
axs[0].grid(True)
axs[0].legend()
# # Secondary axis: Error
# ax0_twin = axs[0].twinx()
# ax0_twin.plot(df['linear_vel_error'], color='black', linestyle=':', linewidth=1.5, label='Linear Velocity Error')
# ax0_twin.set_ylabel('Linear Velocity Error (m/s)', color='black')
# ax0_twin.tick_params(axis='y', labelcolor='black')
# ax0_twin.legend(loc='upper right')

axs[1].plot(df['angular_vel'], color='green', linewidth=2, label='MPC Angular Velocity')
axs[1].plot(df['actual_angular_vel'], color='orange', linestyle='--', linewidth=2, label='Actual Angular Velocity')
axs[1].set_ylabel('Angular Velocity (rad/s)')
axs[1].set_xlabel('Time (s)')
axs[1].set_title('MPC vs. Actual Angular Velocity')
axs[1].grid(True)
axs[1].legend()

# # Secondary axis: Error
# ax1_twin = axs[1].twinx()
# ax1_twin.plot(df['angular_vel_error'], color='black', linestyle=':', linewidth=1.5, label='Angular Velocity Error')
# ax1_twin.set_ylabel('Angular Velocity Error (rad/s)', color='black')
# ax1_twin.tick_params(axis='y', labelcolor='black')
# ax1_twin.legend(loc='upper right')
plt.tight_layout()
plt.show()