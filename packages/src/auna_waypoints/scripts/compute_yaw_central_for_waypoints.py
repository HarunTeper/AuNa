#!/usr/bin/env python3
"""Compute yaw using central-difference (old C++ method) and write extended CSV.

Input formats supported:
1) x,y  (no header) -> output x,y,yaw
2) x,y, (optional trailing comma) handled
3) With header containing x and y (case-insensitive) -> preserves header and adds yaw column.

Yaw is computed with central differences (closed loop by default):
  yaw[i] = atan2(y[i+1] - y[i-1], x[i+1] - x[i-1])
For --open-path: endpoints use forward/backward difference respectively.

Usage:
  ros2 run auna_waypoints compute_yaw_central_for_waypoints -- --input in.csv --output out.csv [--open-path]
OR standalone:
  ./compute_yaw_central_for_waypoints.py -i in.csv -o out.csv
"""
from __future__ import annotations
import csv
import math
import argparse
from pathlib import Path


def angle_wrap_pi(a: float) -> float:
    # Keep final output within [-pi, pi] for consistency
    a = (a + math.pi) % (2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


def compute_yaws_central(xs, ys, open_path: bool):
    n = len(xs)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    yaws = [0.0] * n
    last = n - 1
    for i in range(n):
        if open_path:
            if i == 0:
                dx = xs[1] - xs[0]
                dy = ys[1] - ys[0]
            elif i == last:
                dx = xs[last] - xs[last - 1]
                dy = ys[last] - ys[last - 1]
            else:
                dx = xs[i + 1] - xs[i - 1]
                dy = ys[i + 1] - ys[i - 1]
        else:
            prev_i = (i - 1) % n
            next_i = (i + 1) % n
            dx = xs[next_i] - xs[prev_i]
            dy = ys[next_i] - ys[prev_i]
        yaws[i] = math.atan2(dy, dx)
    # Note: intentionally no unwrap smoothing to mirror the old C++ behavior.
    # Still wrap to [-pi, pi] for a well-defined range.
    return [angle_wrap_pi(y) for y in yaws]


def read_waypoints(path: Path):
    with path.open() as f:
        sample = f.readline()
        f.seek(0)
        has_header = any(h in sample.lower() for h in ("x", "y")) and not all(
            ch.isdigit() or ch in ",.-" for ch in sample.strip()
        )
        reader = csv.reader(f)
        rows = list(reader)
    header = None
    data_rows = rows
    if has_header and rows:
        header = rows[0]
        data_rows = rows[1:]
    xs, ys = [], []
    for r in data_rows:
        if len(r) < 2:
            continue
        try:
            xs.append(float(r[0]))
            ys.append(float(r[1]))
        except ValueError:
            continue
    return header, xs, ys


def write_output(path: Path, header, xs, ys, yaws):
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        if header:
            lower = [h.lower() for h in header]
            if 'yaw' in lower:
                header = [h for h in header if h.lower() != 'yaw']
            w.writerow(header + ['yaw'])
        for x, y, yaw in zip(xs, ys, yaws):
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True,
                    help='Input waypoint CSV (x,y[,yaw])')
    ap.add_argument('-o', '--output', required=True,
                    help='Output CSV with yaw (central-diff)')
    ap.add_argument('--open-path', action='store_true',
                    help='Treat path as open (endpoints use fwd/back diff)')
    args = ap.parse_args()
    header, xs, ys = read_waypoints(Path(args.input))
    if not xs:
        raise SystemExit('No waypoints parsed from input file')
    yaws = compute_yaws_central(xs, ys, args.open_path)
    write_output(Path(args.output), header, xs, ys, yaws)
    print(f"Wrote {len(xs)} waypoints with central-diff yaw to {args.output}")


if __name__ == '__main__':
    main()
