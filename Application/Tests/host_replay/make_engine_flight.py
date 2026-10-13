#!/usr/bin/env python3
"""Synthesize IMU/baro SD-format CSVs for a vertical flight driven by a
recorded engine chamber-pressure log (e.g. a static-fire / VSFT Chamber.csv).

Thrust = k * chamber pressure, with k chosen so the peak pressure gives
--peak-thrust. The vehicle is held on the pad until --release-us (hold-down),
then flies a 1-D vertical trajectory with drag, coasts to apogee, descends
under a parachute at --descent-speed and lands, lying on its side.

Sensors use the firmware mounting (body_x = -sensor_z, body_y = sensor_y,
body_z = sensor_x): 4 IMUs at 6.4 kHz with per-IMU biases, white noise and
engine vibration while the chamber is pressurised; 4 barometers triggered at
20 Hz and delivered 13 ms later.

Model assumptions (mass, Isp, drag, chute) are rough: the output is meant to
exercise the liftoff/apogee/touchdown logic with a realistic thrust profile,
not to predict the trajectory.
"""
import argparse

import numpy as np
import pandas as pd

G = 9.80665


def isa_pressure(h_m):
    return 101325.0 * (1.0 - 2.25577e-5 * h_m) ** 5.25588


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("chamber_csv", help="ts_us,pressure,... chamber log (bar)")
    ap.add_argument("out_dir")
    ap.add_argument("--release-us", type=int, required=True, help="hold-down release time")
    ap.add_argument("--pre-s", type=float, default=240.0, help="pad time before release")
    ap.add_argument("--ground-s", type=float, default=40.0, help="time on ground after landing")
    ap.add_argument("--peak-thrust", type=float, default=7500.0)
    ap.add_argument("--mass", type=float, default=130.0)
    ap.add_argument("--isp", type=float, default=230.0)
    ap.add_argument("--cd", type=float, default=0.5)
    ap.add_argument("--diameter", type=float, default=0.2)
    ap.add_argument("--descent-speed", type=float, default=7.0)
    ap.add_argument("--ground-msl", type=float, default=150.0)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    ch = pd.read_csv(a.chamber_csv)
    ch_t = ch.ts_us.to_numpy(np.int64)
    ch_p = np.clip(ch.pressure.to_numpy(float), 0.0, None)
    k_thrust = a.peak_thrust / ch_p.max()
    area = np.pi * (a.diameter / 2) ** 2

    # ---- 1-D trajectory at 1 kHz ---------------------------------------
    dt = 1e-3
    t0 = a.release_us - int(a.pre_s * 1e6)
    t = [];  h = [];  v = [];  f_x = [];  phase = []
    m, hh, vv, tt = a.mass, 0.0, 0.0, t0
    released = apogee = landed = False
    t_apogee = t_land = None
    while True:
        p_c = np.interp(tt, ch_t, ch_p, left=0.0, right=0.0)
        thrust = k_thrust * p_c
        if tt >= a.release_us:
            released = True
        if not released:
            fx, ph = G, 0                      # on the pad: normal force
        elif not apogee:
            drag = 0.5 * 1.2 * vv * abs(vv) * a.cd * area
            acc = (thrust - drag) / m - G
            m -= thrust / (a.isp * G) * dt
            vv += acc * dt;  hh += vv * dt
            fx, ph = (thrust - drag) / m, 1
            if vv <= 0.0 and tt > a.release_us + 1e6:
                apogee, t_apogee = True, tt
        elif not landed:
            # Parachute: first-order approach to the descent speed.
            vv += (-a.descent_speed - vv) * dt / 1.0 if tt > t_apogee + 1.5e6 else -G * dt
            hh += vv * dt
            fx, ph = (G + (-a.descent_speed - vv) / 1.0) if tt > t_apogee + 1.5e6 else 0.0, 2
            if hh <= 0.0:
                landed, t_land, hh, vv = True, tt, 0.0, 0.0
        else:
            fx, ph = G, 3
            if tt > t_land + a.ground_s * 1e6:
                break
        t.append(tt);  h.append(hh);  v.append(vv);  f_x.append(fx);  phase.append(ph)
        tt += int(dt * 1e6)
    t = np.array(t, np.int64);  h = np.array(h);  f_x = np.array(f_x);  phase = np.array(phase)
    i_bo = np.argmax((t > a.release_us) & (np.interp(t, ch_t, ch_p) < 1.0) & (np.array(v) > 0))
    print(f"release {a.release_us/1e6:.3f}s  burnout ~{t[i_bo]/1e6:.3f}s v={v[i_bo]:.1f} m/s h={h[i_bo]:.1f} m")
    print(f"apogee {t_apogee/1e6:.3f}s h={h.max():.1f} m  landing {t_land/1e6:.3f}s  end {t[-1]/1e6:.3f}s")

    # ---- IMUs: 4 x 6.4 kHz ----------------------------------------------
    period = 1e6 / 6400.0
    rows = []
    for i in range(4):
        ts = (t[0] + i * 40 + np.arange(int((t[-1] - t[0] - 200) / period)) * period).astype(np.int64)
        fxi = np.interp(ts, t, f_x)
        ph = np.interp(ts, t, phase).round()
        f_body = np.zeros((len(ts), 3))
        f_body[:, 0] = fxi
        lying = ph == 3
        f_body[lying, 0] = 0.0
        f_body[lying, 1] = -G                      # on its side: up = -body_y
        pc = np.interp(ts, ch_t, ch_p, left=0.0, right=0.0)
        vib = np.where(pc > 1.0, 4.0, 0.0)[:, None] * rng.standard_normal((len(ts), 3))
        swing = np.where(ph == 2, 0.5, 0.0)[:, None] * rng.standard_normal((len(ts), 3))
        f_body += vib + swing + rng.normal(0, 0.15, 3) + 0.04 * rng.standard_normal((len(ts), 3))
        gyro = rng.normal(0, 0.003, 3) + 0.004 * rng.standard_normal((len(ts), 3))
        gyro += np.where(ph == 2, 0.3, 0.0)[:, None] * rng.standard_normal((len(ts), 3))
        # body -> sensor: sensor_x = body_z, sensor_y = body_y, sensor_z = -body_x
        acc_s = np.stack([f_body[:, 2], f_body[:, 1], -f_body[:, 0]], axis=1)
        gyr_s = np.stack([gyro[:, 2], gyro[:, 1], -gyro[:, 0]], axis=1)
        rows.append(pd.DataFrame({"ts_us": ts, "sensor_index": i,
                                  "ax": acc_s[:, 0], "ay": acc_s[:, 1], "az": acc_s[:, 2],
                                  "gx": gyr_s[:, 0], "gy": gyr_s[:, 1], "gz": gyr_s[:, 2],
                                  "temperature": 306.15, "timestamp_us": ts}))
    imu = pd.concat(rows).sort_values("ts_us", kind="stable")
    imu.to_csv(f"{a.out_dir}/imu.csv", index=False, header=False, float_format="%.5f")

    # ---- Barometers: 4 x 20 Hz ---------------------------------------------
    trig = np.arange(t[0], t[-1] - 20000, 50000, dtype=np.int64)
    hb = np.interp(trig, t, h)
    brows = []
    for i in range(4):
        p = isa_pressure(a.ground_msl + hb) + rng.normal(0, 15) + rng.normal(0, 1.5, len(trig))
        brows.append(pd.DataFrame({"ts_us": trig + 13000 + i * 20, "sensor_index": i,
                                   "pressure_pa": p, "temperature_c": 30.0, "timestamp_us": trig}))
    baro = pd.concat(brows).sort_values("ts_us", kind="stable")
    baro.to_csv(f"{a.out_dir}/baro.csv", index=False, header=False, float_format="%.3f")
    print(f"wrote {len(imu)} IMU rows, {len(baro)} baro rows to {a.out_dir}")


if __name__ == "__main__":
    main()
