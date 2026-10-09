#!/usr/bin/env python3
"""Summarise the [PERF], [IMU-ACQ] and [RADIO] report lines of a serial log.

Usage: perf_summary.py LOG [LOG...] [--skip N]

--skip N drops the first N reports of each log (default 15, i.e. ~15 s at the
default 1 s report period), so that boot and IMU start-up are not counted.
Lines are matched anywhere in a log line, so prefixes added by the capture
tool do not matter.
"""
import re
import sys

KV = re.compile(r"(\w+)=(\d+)")


def parse_perf(line):
    head, _, sections = line.partition("|")
    d = {k: int(v) for k, v in KV.findall(head)}
    d["sections"] = {k: int(v) for k, v in KV.findall(sections)}
    return d


def parse_imu(line):
    imus = {}
    for idx, body in re.findall(r" (\d):(\S+(?: \S+=\S+)*?)(?= \d:|$)", line):
        imus[int(idx)] = {k: int(v) for k, v in KV.findall(body)}
    return imus


def parse_radio(line):
    tx, _, rx = line.partition("|")
    d = {"tx_" + k: int(v) for k, v in KV.findall(tx)}
    d.update({"rx_" + k: int(v) for k, v in KV.findall(rx)})
    d["tx_on"] = "tx=on" in tx
    d["rx_on"] = "rx=on" in rx
    return d


def summarise(path, skip):
    perf, imu, radio, gps, bench, sd, baro, clock = [], [], [], [], [], [], [], []
    with open(path, errors="replace") as f:
        for raw in f:
            line = raw.strip()
            i = line.find("[PERF]")
            if i >= 0:
                perf.append(parse_perf(line[i + 6:]))
                continue
            i = line.find("[IMU-ACQ]")
            if i >= 0:
                imu.append(parse_imu(line[i + 9:]))
                continue
            i = line.find("[RADIO] tx=")
            if i >= 0:
                radio.append(parse_radio(line[i + 8:]))
            if "[GPS] bytes=" in line:
                gps.append({k: int(v) for k, v in KV.findall(line)})
            if "[BENCH] ms=" in line:
                bench.append({k: int(v) for k, v in KV.findall(line)})
            if "[SD] wr=" in line:
                sd.append(line)
            if "[BARO-ACQ]" in line:
                baro.append({k: [int(v) for v in values.split(',')]
                             for k, values in re.findall(r"(read|trigger|healthy)=([\d,]+)", line)})
            if "[IMU-CLOCK]" in line and "write=" in line:
                clock.append([int(v) for v in re.search(r"write=([\d,]+)", line)[1].split(',')])
    perf, imu, radio = perf[skip:], imu[skip:], radio[skip:]
    gps, bench = gps[skip:], bench[skip:]
    baro = baro[skip:]
    clock = clock[skip:]

    print(f"== {path}: {len(perf)} reports after skipping {skip}")
    if perf:
        # ">5ms=3" parses as key "5ms" (the regex skips the ">").
        loop_max = sorted(p["max"] for p in perf)
        stalls = {t: sum(p.get(t, 0) for p in perf) for t in ("5ms", "16ms", "64ms")}
        print(f"loop: avg {sum(p['avg'] for p in perf) / len(perf):.0f} us, "
              f"max per report median {loop_max[len(loop_max) // 2]} us, worst {loop_max[-1]} us; "
              f"stalls >5ms {stalls['5ms']}, >16ms {stalls['16ms']}, >64ms {stalls['64ms']}")
        names = perf[0]["sections"].keys()
        worst = {n: max(p["sections"].get(n, 0) for p in perf) for n in names}
        print("section worst (us): " + " ".join(f"{n}={v}" for n, v in worst.items()))
        if len(perf) > 1 and "ms" in perf[0]:
            print(f"measured duration: {(perf[-1]['ms'] - perf[0]['ms']) / 1000:.3f}s")
    if imu:
        for idx in sorted(imu[0]):
            rows = [r[idx] for r in imu if idx in r]
            fr = sum(r.get("fr", 0) for r in rows) / len(rows)
            lost = sum(r.get("lost", 0) for r in rows)
            gaps = sum(r.get("gap", 0) for r in rows)
            print(f"imu {idx}: {fr:.0f} frames/report, gaps {gaps}, lost {lost}, "
                  f"max dt {max(r.get('dt', 0) for r in rows)} us, "
                  f"fifo hwm {max(r.get('hwm', 0) for r in rows)} frames, "
                  f"capped {sum(r.get('cap', 0) for r in rows)}, "
                  f"full {sum(r.get('full', 0) for r in rows)}, "
                  f"app overwrites {sum(r.get('app_drop', 0) for r in rows)}")
            if fr == 0:
                print(f"  FAIL: imu {idx} produced no frames; zero loss is not a pass")
            if len(imu) == len(perf) and len(perf) > 1 and "ms" in perf[0]:
                seconds = (perf[-1]["ms"] - perf[0]["ms"]) / 1000.0
                frames = sum(r[idx].get("fr", 0) for r in imu[1:] if idx in r)
                if len(clock) == len(perf):
                    frames = clock[-1][idx] - clock[0][idx]
                if seconds > 0:
                    print(f"  normalised rate: {frames / seconds:.1f} frames/s over {seconds:.3f}s")
                print(f"  cumulative loss delta: {rows[-1].get('tot', 0) - rows[0].get('tot', 0)}")
                if "app_tot" in rows[-1]:
                    print(f"  cumulative app overwrite delta: {rows[-1]['app_tot'] - rows[0].get('app_tot', 0)}")
    if radio:
        tot = lambda k: sum(r.get(k, 0) for r in radio)
        print(f"radio tx: on {sum(r['tx_on'] for r in radio)}/{len(radio)}, "
              f"started {tot('tx_start')}, sent {tot('tx_sent')}, busy {tot('tx_busy')}, "
              f"absent {tot('tx_absent')}, timeouts {tot('tx_tmo')}, reconf {tot('tx_reconf')}, "
              f"max air {max(r.get('tx_air', 0) for r in radio)} ms")
        print(f"radio rx: on {sum(r['rx_on'] for r in radio)}/{len(radio)}, "
              f"packets {tot('rx_pkt')}, reconf {tot('rx_reconf')}")
    if gps:
        print(f"gps: bytes/report {sum(r.get('bytes', 0) for r in gps)/len(gps):.0f}, "
              f"pvt/report {sum(r.get('pvt', 0) for r in gps)/len(gps):.2f}, "
              f"overruns {sum(r.get('ore', 0) for r in gps)}")
    if bench:
        print(f"estimator: states {sorted(set(r.get('state') for r in bench))}, "
              f"drop delta {bench[-1].get('drop', 0) - bench[0].get('drop', 0)}, "
              f"max lag {max(r.get('behind', 0) for r in bench)} us, "
              f"diverged reports {sum(r.get('div', 0) for r in bench)}, "
              f"liftoff reports {sum(r.get('liftoff', 0) for r in bench)}, "
              f"apogee reports {sum(r.get('apogee', 0) for r in bench)}")
    if sd:
        print(f"sd last: {sd[-1]}")
    if baro:
        for idx in range(len(baro[0]["read"])):
            reads = baro[-1]["read"][idx] - baro[0]["read"][idx]
            triggers = baro[-1]["trigger"][idx] - baro[0]["trigger"][idx]
            healthy = sum(r["healthy"][idx] for r in baro)
            print(f"baro {idx}: read delta {reads}, trigger delta {triggers}, "
                  f"healthy {healthy}/{len(baro)} reports")


def main(argv):
    skip = 15
    paths = []
    it = iter(argv)
    for a in it:
        if a == "--skip":
            skip = int(next(it))
        else:
            paths.append(a)
    if not paths:
        print(__doc__)
        return 1
    for p in paths:
        summarise(p, skip)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
