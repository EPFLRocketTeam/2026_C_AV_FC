#!/usr/bin/env python3
"""Flight test data plotting from decoded SD logs."""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

sns.set_theme(style="darkgrid")
RESULTS = Path("results")
PLOTS = Path("plots")
PLOTS.mkdir(exist_ok=True)

LIFTOFF_US = 1350346034
BURN_END_US = 1352346716  # BURN->ASCENT
APOGEE_US = 1356068941    # ASCENT->DESCENT

def t_flight(ts_us):
    """Convert absolute timestamp to seconds after liftoff."""
    return (ts_us - LIFTOFF_US) / 1e6

# ============================================================
# 1. ESKF State: Position, Velocity, Attitude
# ============================================================
print("Loading ESKF State...")
state = pd.read_csv(RESULTS / "eskf/State.csv")
state['t'] = t_flight(state['ts_us'])

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

# Altitude (NED: z is down, so -p[2] = altitude)
axes[0].plot(state['t'], -state['p[2]'], 'b-', linewidth=0.8)
axes[0].axvline(0, color='r', linestyle='--', alpha=0.7, label='Liftoff')
axes[0].axvline(t_flight(BURN_END_US), color='orange', linestyle='--', alpha=0.7, label='Burnout')
axes[0].axvline(t_flight(APOGEE_US), color='green', linestyle='--', alpha=0.7, label='Apogee')
axes[0].set_ylabel('Altitude [m]')
axes[0].legend()
axes[0].set_title('ESKF Navigation State')

# Velocity
axes[1].plot(state['t'], state['v[0]'], label='Vx (North)')
axes[1].plot(state['t'], state['v[1]'], label='Vy (East)')
axes[1].plot(state['t'], -state['v[2]'], label='Vz (Up)')
axes[1].set_ylabel('Velocity [m/s]')
axes[1].legend()

# Horizontal position
axes[2].plot(state['t'], state['p[0]'], label='North')
axes[2].plot(state['t'], state['p[1]'], label='East')
axes[2].set_ylabel('Position [m]')
axes[2].set_xlabel('Time after liftoff [s]')
axes[2].legend()

plt.tight_layout()
plt.savefig(PLOTS / "01_eskf_state.png", dpi=150)
plt.close()
print("  -> 01_eskf_state.png")

# ============================================================
# 2. Flight Shadow: Altitude & Velocity (onboard flight logic view)
# ============================================================
print("Loading Flight Shadow...")
fs = pd.read_csv(RESULTS / "eskf/FlightShadow.csv")
fs['t'] = t_flight(fs['ts_us'])

fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
axes[0].plot(fs['t'], fs['altitude_m'], 'b-', linewidth=0.8)
axes[0].axvline(0, color='r', linestyle='--', alpha=0.5)
axes[0].set_ylabel('Altitude [m]')
axes[0].set_title('Flight Shadow (Onboard Flight Logic)')

axes[1].plot(fs['t'], fs['velocity_mps'], 'r-', linewidth=0.8)
axes[1].axvline(0, color='r', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Velocity [m/s]')
axes[1].set_xlabel('Time after liftoff [s]')

plt.tight_layout()
plt.savefig(PLOTS / "02_flight_shadow.png", dpi=150)
plt.close()
print("  -> 02_flight_shadow.png")

# ============================================================
# 3. IMU Dynamics (body frame accelerations & angular rates)
# ============================================================
print("Loading IMU Dynamics...")
imu_dyn = pd.read_csv(RESULTS / "eskf/ImuDynamics.csv")
imu_dyn['t'] = t_flight(imu_dyn['ts_us'])

fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

axes[0].plot(imu_dyn['t'], imu_dyn['accel_body[0]'], label='Ax (fwd)', linewidth=0.5, alpha=0.8)
axes[0].plot(imu_dyn['t'], imu_dyn['accel_body[1]'], label='Ay (right)', linewidth=0.5, alpha=0.8)
axes[0].plot(imu_dyn['t'], imu_dyn['accel_body[2]'], label='Az (down)', linewidth=0.5, alpha=0.8)
axes[0].set_ylabel('Accel [m/s²]')
axes[0].set_title('IMU Dynamics (Body Frame)')
axes[0].legend()

axes[1].plot(imu_dyn['t'], np.degrees(imu_dyn['gyro_body[0]']), label='Gx (roll)', linewidth=0.5, alpha=0.8)
axes[1].plot(imu_dyn['t'], np.degrees(imu_dyn['gyro_body[1]']), label='Gy (pitch)', linewidth=0.5, alpha=0.8)
axes[1].plot(imu_dyn['t'], np.degrees(imu_dyn['gyro_body[2]']), label='Gz (yaw)', linewidth=0.5, alpha=0.8)
axes[1].set_ylabel('Gyro [deg/s]')
axes[1].set_xlabel('Time after liftoff [s]')
axes[1].legend()

plt.tight_layout()
plt.savefig(PLOTS / "03_imu_dynamics.png", dpi=150)
plt.close()
print("  -> 03_imu_dynamics.png")

# ============================================================
# 4. Barometer data
# ============================================================
print("Loading Baro Samples...")
baro = pd.read_csv(RESULTS / "sensors/BaroSample.csv")
baro['t'] = t_flight(baro['ts_us'])
# Only flight window
baro_flight = baro[(baro['t'] >= -5) & (baro['t'] <= 30)]

fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
for si in range(4):
    subset = baro_flight[baro_flight['sensor_index'] == si]
    axes[0].plot(subset['t'], subset['pressure_pa'], label=f'Baro {si}', linewidth=0.7, alpha=0.8)
axes[0].set_ylabel('Pressure [Pa]')
axes[0].set_title('Barometer Readings (Flight Window)')
axes[0].legend()
axes[0].axvline(0, color='r', linestyle='--', alpha=0.5)

for si in range(4):
    subset = baro_flight[baro_flight['sensor_index'] == si]
    axes[1].plot(subset['t'], subset['temperature_c'], label=f'Baro {si}', linewidth=0.7, alpha=0.8)
axes[1].set_ylabel('Temperature [°C]')
axes[1].set_xlabel('Time after liftoff [s]')
axes[1].legend()

plt.tight_layout()
plt.savefig(PLOTS / "04_baro_flight.png", dpi=150)
plt.close()
print("  -> 04_baro_flight.png")

# ============================================================
# 5. App Metrics: Loop timing & SD health
# ============================================================
print("Loading App Metrics...")
metrics = pd.read_csv(RESULTS / "AppMetrics.csv")
metrics['t'] = t_flight(metrics['ts_us'])
# Focus on flight window
mf = metrics[(metrics['t'] >= -10) & (metrics['t'] <= 30)]

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

axes[0].plot(mf['t'], mf['loop_avg_us']/1000, 'b-', label='Loop avg', linewidth=0.8)
axes[0].plot(mf['t'], mf['loop_max_us']/1000, 'r-', label='Loop max', linewidth=0.8, alpha=0.6)
axes[0].set_ylabel('Loop time [ms]')
axes[0].set_title('System Performance (Flight Window)')
axes[0].legend()
axes[0].axvline(0, color='r', linestyle='--', alpha=0.5)

axes[1].plot(mf['t'], mf['kalman_avg_us']/1000, 'g-', label='Kalman avg', linewidth=0.8)
axes[1].plot(mf['t'], mf['kalman_max_us']/1000, 'r-', label='Kalman max', linewidth=0.8, alpha=0.6)
axes[1].set_ylabel('Kalman time [ms]')
axes[1].legend()

axes[2].plot(mf['t'], mf['group_fire_count'], 'b-', label='Group fires', linewidth=0.8)
axes[2].plot(mf['t'], mf['solo_flush_count'], 'r-', label='Solo flushes', linewidth=0.8, alpha=0.6)
axes[2].set_ylabel('Count / interval')
axes[2].set_xlabel('Time after liftoff [s]')
axes[2].legend()

plt.tight_layout()
plt.savefig(PLOTS / "05_app_metrics_flight.png", dpi=150)
plt.close()
print("  -> 05_app_metrics_flight.png")

# ============================================================
# 6. SD Health
# ============================================================
print("Loading SD Health...")
sd = pd.read_csv(RESULTS / "SdHealth.csv")
sd['t'] = t_flight(sd['ts_us'])
sd_f = sd[(sd['t'] >= -10) & (sd['t'] <= 30)]

fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

axes[0].plot(sd_f['t'], sd_f['arena_used_bytes']/1024, 'b-', linewidth=0.8)
axes[0].axhline(sd_f['arena_total_bytes'].iloc[0]/1024, color='r', linestyle='--', alpha=0.5, label='Arena capacity')
axes[0].set_ylabel('Arena used [KB]')
axes[0].set_title('SD Card Health (Flight Window)')
axes[0].legend()
axes[0].axvline(0, color='r', linestyle='--', alpha=0.3)

sd['write_rate_kbps'] = sd['bytes_written'].diff() / sd['ts_us'].diff() * 1e6 / 1024
sd_f2 = sd[(sd['t'] >= -10) & (sd['t'] <= 30)]
axes[1].plot(sd_f2['t'], sd_f2['write_rate_kbps'], 'g-', linewidth=0.8)
axes[1].set_ylabel('Write rate [KB/s]')
axes[1].set_xlabel('Time after liftoff [s]')

plt.tight_layout()
plt.savefig(PLOTS / "06_sd_health_flight.png", dpi=150)
plt.close()
print("  -> 06_sd_health_flight.png")

# ============================================================
# 7. Euler angles from quaternion
# ============================================================
print("Computing Euler angles...")
q0 = state['q[0]'].values
q1 = state['q[1]'].values
q2 = state['q[2]'].values
q3 = state['q[3]'].values

# Roll, Pitch, Yaw from quaternion (NED frame)
roll = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
yaw = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
axes[0].plot(state['t'], roll, 'r-', linewidth=0.7)
axes[0].set_ylabel('Roll [deg]')
axes[0].set_title('ESKF Attitude (Euler Angles)')
axes[0].axvline(0, color='k', linestyle='--', alpha=0.3)

axes[1].plot(state['t'], pitch, 'g-', linewidth=0.7)
axes[1].set_ylabel('Pitch [deg]')

axes[2].plot(state['t'], yaw, 'b-', linewidth=0.7)
axes[2].set_ylabel('Yaw [deg]')
axes[2].set_xlabel('Time after liftoff [s]')

plt.tight_layout()
plt.savefig(PLOTS / "07_attitude_euler.png", dpi=150)
plt.close()
print("  -> 07_attitude_euler.png")

# ============================================================
# 8. Covariance (if available)
# ============================================================
print("Loading Covariance...")
try:
    cov = pd.read_csv(RESULTS / "eskf/Covariance.csv")
    cov['t'] = t_flight(cov['ts_us'])
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    # Position covariance sqrt (1-sigma)
    pos_cols = [c for c in cov.columns if c.startswith('P_pos')]
    vel_cols = [c for c in cov.columns if c.startswith('P_vel')]
    att_cols = [c for c in cov.columns if c.startswith('P_att')]
    
    if pos_cols:
        for c in pos_cols:
            axes[0].plot(cov['t'], np.sqrt(cov[c]), linewidth=0.7, label=c)
        axes[0].set_ylabel('Pos 1σ [m]')
        axes[0].set_title('ESKF Covariance (1σ)')
        axes[0].legend()
    
    if vel_cols:
        for c in vel_cols:
            axes[1].plot(cov['t'], np.sqrt(cov[c]), linewidth=0.7, label=c)
        axes[1].set_ylabel('Vel 1σ [m/s]')
        axes[1].legend()
    
    if att_cols:
        for c in att_cols:
            axes[2].plot(cov['t'], np.degrees(np.sqrt(cov[c])), linewidth=0.7, label=c)
        axes[2].set_ylabel('Att 1σ [deg]')
        axes[2].legend()
    
    axes[2].set_xlabel('Time after liftoff [s]')
    plt.tight_layout()
    plt.savefig(PLOTS / "08_covariance.png", dpi=150)
    plt.close()
    print("  -> 08_covariance.png")
except Exception as e:
    print(f"  Covariance plot skipped: {e}")

# ============================================================
# 9. 3D trajectory
# ============================================================
print("Generating 3D trajectory...")
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.plot(state['p[0]'], state['p[1]'], -state['p[2]'], 'b-', linewidth=0.8)
ax.scatter([0], [0], [0], color='green', s=100, label='Liftoff')
apogee_idx = (-state['p[2]']).idxmax()
ax.scatter([state['p[0]'].iloc[apogee_idx]], [state['p[1]'].iloc[apogee_idx]], 
           [-state['p[2]'].iloc[apogee_idx]], color='red', s=100, label='Apogee')
ax.set_xlabel('North [m]')
ax.set_ylabel('East [m]')
ax.set_zlabel('Altitude [m]')
ax.set_title('3D Flight Trajectory')
ax.legend()
plt.tight_layout()
plt.savefig(PLOTS / "09_trajectory_3d.png", dpi=150)
plt.close()
print("  -> 09_trajectory_3d.png")

# ============================================================
# 10. IMU raw sensor comparison (flight window, decimated for plotting)
# ============================================================
print("Loading IMU samples (flight window)...")
# Only load flight window to keep memory manageable
imu_chunks = []
for chunk in pd.read_csv(RESULTS / "sensors/IMUSample.csv", chunksize=2_000_000):
    flight = chunk[(chunk['ts_us'] >= LIFTOFF_US - 1_000_000) & 
                   (chunk['ts_us'] <= LIFTOFF_US + 10_000_000)]
    if len(flight) > 0:
        imu_chunks.append(flight)
    if len(chunk) > 0 and chunk['ts_us'].iloc[-1] > LIFTOFF_US + 10_000_000:
        break

imu_flight = pd.concat(imu_chunks) if imu_chunks else pd.DataFrame()

if len(imu_flight) > 0:
    imu_flight['t'] = t_flight(imu_flight['ts_us'])
    # Decimate for plotting (every 64th sample)
    imu_plot = imu_flight.iloc[::64]
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for si in range(4):
        s = imu_plot[imu_plot['sensor_index'] == si]
        axes[0].plot(s['t'], s['sensor_data.accel_x'], linewidth=0.5, alpha=0.7, label=f'IMU{si}')
    axes[0].set_ylabel('Accel X [m/s²]')
    axes[0].set_title('Raw IMU Axial Acceleration (First 10s of Flight)')
    axes[0].legend()
    axes[0].axvline(0, color='r', linestyle='--', alpha=0.5)
    
    for si in range(4):
        s = imu_plot[imu_plot['sensor_index'] == si]
        axes[1].plot(s['t'], s['sensor_data.accel_z'], linewidth=0.5, alpha=0.7, label=f'IMU{si}')
    axes[1].set_ylabel('Accel Z [m/s²]')
    axes[1].set_xlabel('Time after liftoff [s]')
    axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(PLOTS / "10_imu_raw_flight.png", dpi=150)
    plt.close()
    print("  -> 10_imu_raw_flight.png")

# ============================================================
# 11. Bias estimates
# ============================================================
print("Plotting bias estimates...")
fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

axes[0].plot(state['t'], state['b_acc[0]'], label='b_acc_x')
axes[0].plot(state['t'], state['b_acc[1]'], label='b_acc_y')
axes[0].plot(state['t'], state['b_acc[2]'], label='b_acc_z')
axes[0].set_ylabel('Accel bias [m/s²]')
axes[0].set_title('ESKF Bias Estimates')
axes[0].legend()

axes[1].plot(state['t'], np.degrees(state['b_gyro[0]']), label='b_gyro_x')
axes[1].plot(state['t'], np.degrees(state['b_gyro[1]']), label='b_gyro_y')
axes[1].plot(state['t'], np.degrees(state['b_gyro[2]']), label='b_gyro_z')
axes[1].set_ylabel('Gyro bias [deg/s]')
axes[1].set_xlabel('Time after liftoff [s]')
axes[1].legend()

plt.tight_layout()
plt.savefig(PLOTS / "11_bias_estimates.png", dpi=150)
plt.close()
print("  -> 11_bias_estimates.png")

print("\nAll plots saved to other/plots/")
