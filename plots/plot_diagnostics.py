#!/usr/bin/env python3
"""Pre-parachute plots and Kalman divergence diagnostics."""
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
BURN_END_US = 1352346716
APOGEE_US = 1356068941
CUTOFF_T = 7.0  # seconds after liftoff - before parachute effects

def t_flight(ts_us):
    return (ts_us - LIFTOFF_US) / 1e6

# ============================================================
# Load data
# ============================================================
state = pd.read_csv(RESULTS / "eskf/State.csv")
state['t'] = t_flight(state['ts_us'])

fs = pd.read_csv(RESULTS / "eskf/FlightShadow.csv")
fs['t'] = t_flight(fs['ts_us'])

cov = pd.read_csv(RESULTS / "eskf/Covariance.csv")
cov['t'] = t_flight(cov['ts_us'])

APOGEE_T = t_flight(APOGEE_US)
BURNOUT_T = t_flight(BURN_END_US)

# ============================================================
# Pre-parachute filtered data
# ============================================================
state_pre = state[state['t'] <= CUTOFF_T]
fs_pre = fs[fs['t'] <= CUTOFF_T]
cov_pre = cov[cov['t'] <= CUTOFF_T]

# ============================================================
# 1. ESKF State (pre-parachute)
# ============================================================
print("Generating pre-parachute ESKF state...")
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

axes[0].plot(state_pre['t'], -state_pre['p[2]'], 'b-', linewidth=0.8)
axes[0].axvline(0, color='r', linestyle='--', alpha=0.7, label='Liftoff')
axes[0].axvline(BURNOUT_T, color='orange', linestyle='--', alpha=0.7, label='Burnout')
axes[0].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.7, label='Apogee detected')
axes[0].set_ylabel('Altitude [m]')
axes[0].legend()
axes[0].set_title('ESKF State (Pre-Parachute, T < 7s)')

axes[1].plot(state_pre['t'], state_pre['v[0]'], label='Vx (North)')
axes[1].plot(state_pre['t'], state_pre['v[1]'], label='Vy (East)')
axes[1].plot(state_pre['t'], -state_pre['v[2]'], label='Vz (Up)')
axes[1].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Velocity [m/s]')
axes[1].legend()

axes[2].plot(state_pre['t'], state_pre['p[0]'], label='North')
axes[2].plot(state_pre['t'], state_pre['p[1]'], label='East')
axes[2].set_ylabel('Position [m]')
axes[2].set_xlabel('Time after liftoff [s]')
axes[2].legend()

plt.tight_layout()
plt.savefig(PLOTS / "12_eskf_state_pre_chute.png", dpi=150)
plt.close()
print("  -> 12_eskf_state_pre_chute.png")

# ============================================================
# 2. Flight Shadow (pre-parachute)
# ============================================================
print("Generating pre-parachute flight shadow...")
fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
axes[0].plot(fs_pre['t'], fs_pre['altitude_m'], 'b-', linewidth=0.8)
axes[0].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.7, label='Apogee detected')
axes[0].set_ylabel('Altitude [m]')
axes[0].set_title('Flight Shadow (Pre-Parachute)')
axes[0].legend()

axes[1].plot(fs_pre['t'], fs_pre['velocity_mps'], 'r-', linewidth=0.8)
axes[1].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Velocity [m/s]')
axes[1].set_xlabel('Time after liftoff [s]')

plt.tight_layout()
plt.savefig(PLOTS / "13_flight_shadow_pre_chute.png", dpi=150)
plt.close()
print("  -> 13_flight_shadow_pre_chute.png")

# ============================================================
# 3. Attitude (pre-parachute)
# ============================================================
print("Generating pre-parachute attitude...")
q0 = state_pre['q[0]'].values
q1 = state_pre['q[1]'].values
q2 = state_pre['q[2]'].values
q3 = state_pre['q[3]'].values

roll = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
pitch = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
yaw = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
axes[0].plot(state_pre['t'], roll, 'r-', linewidth=0.7)
axes[0].set_ylabel('Roll [deg]')
axes[0].set_title('Attitude (Pre-Parachute)')
axes[0].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.5)

axes[1].plot(state_pre['t'], pitch, 'g-', linewidth=0.7)
axes[1].set_ylabel('Pitch [deg]')

axes[2].plot(state_pre['t'], yaw, 'b-', linewidth=0.7)
axes[2].set_ylabel('Yaw [deg]')
axes[2].set_xlabel('Time after liftoff [s]')

plt.tight_layout()
plt.savefig(PLOTS / "14_attitude_pre_chute.png", dpi=150)
plt.close()
print("  -> 14_attitude_pre_chute.png")

# ============================================================
# 4. Covariance (pre-parachute)
# ============================================================
print("Generating pre-parachute covariance...")
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

for i in range(3):
    axes[0].plot(cov_pre['t'], np.sqrt(cov_pre[f'P_diag[{i}]']), label=f'pos[{i}]', linewidth=0.7)
axes[0].set_ylabel('Position 1σ [m]')
axes[0].set_title('Covariance (Pre-Parachute)')
axes[0].legend()

for i in range(3, 6):
    axes[1].plot(cov_pre['t'], np.sqrt(cov_pre[f'P_diag[{i}]']), label=f'vel[{i-3}]', linewidth=0.7)
axes[1].set_ylabel('Velocity 1σ [m/s]')
axes[1].legend()

for i in range(6, 9):
    axes[2].plot(cov_pre['t'], np.degrees(np.sqrt(cov_pre[f'P_diag[{i}]'])), label=f'att[{i-6}]', linewidth=0.7)
axes[2].set_ylabel('Attitude 1σ [deg]')
axes[2].set_xlabel('Time after liftoff [s]')
axes[2].legend()

plt.tight_layout()
plt.savefig(PLOTS / "15_covariance_pre_chute.png", dpi=150)
plt.close()
print("  -> 15_covariance_pre_chute.png")

# ============================================================
# 5. 3D trajectory (pre-parachute)
# ============================================================
print("Generating pre-parachute 3D trajectory...")
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.plot(state_pre['p[0]'], state_pre['p[1]'], -state_pre['p[2]'], 'b-', linewidth=1.2)
ax.scatter([0], [0], [0], color='green', s=100, label='Liftoff')
apogee_idx = (-state_pre['p[2]']).idxmax()
ax.scatter([state_pre['p[0]'].iloc[apogee_idx]], [state_pre['p[1]'].iloc[apogee_idx]],
           [-state_pre['p[2]'].iloc[apogee_idx]], color='red', s=100, label='Apogee')
ax.set_xlabel('North [m]')
ax.set_ylabel('East [m]')
ax.set_zlabel('Altitude [m]')
ax.set_title('3D Trajectory (Pre-Parachute)')
ax.legend()
plt.tight_layout()
plt.savefig(PLOTS / "16_trajectory_3d_pre_chute.png", dpi=150)
plt.close()
print("  -> 16_trajectory_3d_pre_chute.png")

# ============================================================
# 6. KALMAN DIVERGENCE DIAGNOSTIC
# ============================================================
print("\n" + "="*60)
print("KALMAN DIVERGENCE DIAGNOSTIC")
print("="*60)

# Load all relevant data
baro = pd.read_csv(RESULTS / "sensors/BaroSample.csv")
baro['t'] = t_flight(baro['ts_us'])

events = pd.read_csv(RESULTS / "eskf/Event.csv")
events['t'] = t_flight(events['ts_us'])

corrections = pd.read_csv(RESULTS / "eskf/Correction.csv")
corrections['t'] = t_flight(corrections['ts_us'])

print(f"\nFSM Transitions:")
fsm = pd.read_csv(RESULTS / "FsmTransition.csv")
fsm['t'] = t_flight(fsm['ts_us'])
print(fsm[['t','prev_state','new_state']].to_string(index=False))

print(f"\nESKF Events:")
print(events.columns.tolist())
print(events[['t'] + [c for c in events.columns if c != 't' and c != 'ts_us']].head(20).to_string())

print(f"\nCorrections:")
print(corrections.columns.tolist())
print(corrections.head(10).to_string())

# State analysis: find divergence onset
print("\n\nSTATE FREEZE ANALYSIS:")
state['timestamp_us_col'] = state['timestamp_us']
state['ts_diff'] = state['ts_us'].diff()
state['internal_diff'] = state['timestamp_us_col'].diff()
# Where internal clock doesn't advance but wall clock does
frozen = state[(state['ts_diff'] > 0) & (state['internal_diff'] == 0)]
print(f"Rows where ESKF internal clock is frozen: {len(frozen)}")
if len(frozen) > 0:
    # Group consecutive
    frozen_t = frozen['t'].values
    groups = []
    start = frozen_t[0]
    prev = frozen_t[0]
    for t in frozen_t[1:]:
        if t - prev > 0.05:
            groups.append((start, prev))
            start = t
        prev = t
    groups.append((start, prev))
    print(f"Frozen periods ({len(groups)} total):")
    for s, e in groups:
        duration_ms = (e - s) * 1000
        print(f"  T+{s:.3f}s to T+{e:.3f}s ({duration_ms:.0f}ms)")
        # What's happening during this freeze?
        if s > 7:
            region = state[(state['t'] >= s-0.1) & (state['t'] <= e+0.1)]
            print(f"    State at freeze start: alt={-region['p[2]'].iloc[0]:.1f}m, Vz={-region['v[2]'].iloc[0]:.1f}m/s")

# Baro analysis around parachute
print("\n\nBARO SPIKE ANALYSIS (parachute deployment):")
baro_s0 = baro[(baro['sensor_index'] == 0) & (baro['t'] > 5) & (baro['t'] < 12)]
baro_s0_pressure = baro_s0['pressure_pa'].values
baro_s0_t = baro_s0['t'].values
# Rate of pressure change
dp_dt = np.gradient(baro_s0_pressure, baro_s0_t)
max_dp_idx = np.argmax(np.abs(dp_dt))
print(f"Max pressure rate of change: {dp_dt[max_dp_idx]:.1f} Pa/s at T+{baro_s0_t[max_dp_idx]:.3f}s")
# Pressure at apogee vs at ground
p_at_apogee = baro_s0[baro_s0['t'].between(5.5, 6.0)]['pressure_pa'].mean()
p_at_ground = baro[(baro['sensor_index']==0) & (baro['t'] < 0)]['pressure_pa'].iloc[-10:].mean()
alt_baro = (p_at_ground - p_at_apogee) / 12.0  # rough estimate 12 Pa/m
print(f"Baro-derived apogee altitude: ~{alt_baro:.0f}m (p_ground={p_at_ground:.0f}, p_apogee={p_at_apogee:.0f})")

# Summary diagnostic plot
print("\nGenerating divergence diagnostic plot...")
fig, axes = plt.subplots(5, 1, figsize=(16, 18), sharex=True)

# Panel 1: Altitude (ESKF vs baro-derived)
axes[0].plot(state['t'], -state['p[2]'], 'b-', linewidth=0.8, label='ESKF altitude')
# Baro-derived altitude for comparison
baro_alt = (p_at_ground - baro[baro['sensor_index']==0]['pressure_pa']) / 12.0
baro_t_s0 = baro[baro['sensor_index']==0]['t']
axes[0].plot(baro_t_s0, baro_alt, 'r-', linewidth=0.5, alpha=0.6, label='Baro-derived alt (~12 Pa/m)')
axes[0].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.7, label='Apogee detected')
axes[0].axvline(CUTOFF_T, color='purple', linestyle='--', alpha=0.7, label='Parachute shock')
axes[0].set_ylabel('Altitude [m]')
axes[0].set_title('Kalman Divergence Diagnostic')
axes[0].legend()
axes[0].set_xlim(-1, 24)

# Panel 2: Velocity
axes[1].plot(state['t'], -state['v[2]'], 'g-', linewidth=0.8, label='ESKF Vz (up)')
axes[1].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.5)
axes[1].axvline(CUTOFF_T, color='purple', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Vertical velocity [m/s]')
axes[1].legend()

# Panel 3: Baro pressure (raw)
for si in range(4):
    b = baro[(baro['sensor_index']==si) & (baro['t'] > -1) & (baro['t'] < 24)]
    axes[2].plot(b['t'], b['pressure_pa'], linewidth=0.5, alpha=0.7, label=f'Baro {si}')
axes[2].axvline(APOGEE_T, color='green', linestyle='--', alpha=0.5)
axes[2].axvline(CUTOFF_T, color='purple', linestyle='--', alpha=0.5)
axes[2].set_ylabel('Pressure [Pa]')
axes[2].legend()

# Panel 4: Covariance position
for i in range(3):
    axes[3].plot(cov['t'], np.sqrt(cov[f'P_diag[{i}]']), linewidth=0.7, label=f'pos[{i}]')
axes[3].axvline(CUTOFF_T, color='purple', linestyle='--', alpha=0.5)
axes[3].set_ylabel('Pos 1σ [m]')
axes[3].legend()

# Panel 5: ESKF internal time lag
state['lag_ms'] = (state['ts_us'] - state['timestamp_us']) / 1000
axes[4].plot(state['t'], state['lag_ms'], 'k-', linewidth=0.7)
axes[4].set_ylabel('ESKF lag [ms]')
axes[4].set_xlabel('Time after liftoff [s]')
axes[4].axvline(CUTOFF_T, color='purple', linestyle='--', alpha=0.5)

plt.tight_layout()
plt.savefig(PLOTS / "17_divergence_diagnostic.png", dpi=150)
plt.close()
print("  -> 17_divergence_diagnostic.png")

print("\nDone!")
