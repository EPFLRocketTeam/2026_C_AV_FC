import sys, pandas as pd
L = 1350346034
for name in sys.argv[2:]:
    r = pd.read_csv(f"{sys.argv[1]}/{name}.csv"); r = r[r.t_us > L - 1e6]
    ev = r[r.apogee == 1].t_us.min()
    e = r.loc[r.eskf_alt_up.idxmax()]; f = r.loc[r.fs_alt_up.idxmax()]
    burn = r[r.t_us < L + 3e6]
    vmax = burn.eskf_vz_up.max(); fvmax = burn.fs_vz_up.max()
    evs = f"{ev/1e6:.3f}s" if ev == ev else "none"
    print(f"{name:22s} apogee evt {evs:10s} ESKF max {e.eskf_alt_up:6.1f} m @ {e.t_us/1e6:.3f}  FS max {f.fs_alt_up:6.1f} m @ {f.t_us/1e6:.3f}  max vz up ESKF {vmax:5.1f} FS {fvmax:5.1f}")
