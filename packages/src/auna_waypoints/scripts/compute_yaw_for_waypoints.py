#!/usr/bin/env python3
"""Compute yaw for a CSV waypoint file and write extended file.

Input formats supported:
1) x,y  (no header) -> output x,y,yaw
2) x,y, (optional trailing comma) handled
3) With header containing x and y (case-insensitive) -> preserves header and adds yaw column.

Yaw is computed using forward difference for all points except the last,
which uses the direction to the first point (assumes loop) unless --open-path.
For --open-path: last uses backward difference, first uses forward difference.

Usage:
  ros2 run auna_waypoints compute_yaw_for_waypoints -- --input in.csv --output out.csv [--open-path]
OR standalone:
  ./compute_yaw_for_waypoints.py -i in.csv -o out.csv
"""
from __future__ import annotations
import csv
import math
import argparse
from pathlib import Path


def angle_wrap_pi(a: float) -> float:
    a = (a + math.pi) % (2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


def compute_yaws(xs, ys, open_path: bool):
    n = len(xs)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    yaws = [0.0] * n
    for i in range(n):
        if i == n - 1:
            if open_path:
                dx = xs[i] - xs[i-1]
                dy = ys[i] - ys[i-1]
            else:
                dx = xs[0] - xs[i]
                dy = ys[0] - ys[i]
        else:
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
        yaws[i] = math.atan2(dy, dx)
    # Optional unwrap then rewrap to smooth heading transitions
    for i in range(1, n):
        diff = yaws[i] - yaws[i-1]
        if diff > math.pi:
            yaws[i] -= 2 * math.pi
        elif diff < -math.pi:
            yaws[i] += 2 * math.pi
    # Finally wrap to [-pi, pi]
    return [angle_wrap_pi(y) for y in yaws]


def read_waypoints(path: Path):
    with path.open() as f:
        sample = f.readline()
        f.seek(0)
        has_header = any(h in sample.lower() for h in ("x", "y")) and not all(
            ch.isdigit() or ch in ",.-" for ch in sample.strip())
        reader = csv.reader(f)
        rows = list(reader)
    header = None
    data_rows = rows
    if has_header:
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
            # remove existing yaw if present
            lower = [h.lower() for h in header]
            if 'yaw' in lower:
                # rebuild header without old yaw
                header = [h for h in header if h.lower() != 'yaw']
            w.writerow(header + ['yaw'])
        for x, y, yaw in zip(xs, ys, yaws):
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-i', '--input', required=True,
                    help='Input waypoint CSV (x,y[,yaw])')
    ap.add_argument('-o', '--output', required=True,
                    help='Output CSV with yaw')
    ap.add_argument('--open-path', action='store_true',
                    help='Treat path as open instead of loop')
    args = ap.parse_args()
    header, xs, ys = read_waypoints(Path(args.input))
    if not xs:
        raise SystemExit('No waypoints parsed from input file')
    yaws = compute_yaws(xs, ys, args.open_path)
    write_output(Path(args.output), header, xs, ys, yaws)
    print(f"Wrote {len(xs)} waypoints with yaw to {args.output}")


if __name__ == '__main__':
    main()
