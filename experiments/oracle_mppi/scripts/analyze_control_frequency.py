#!/usr/bin/env python3
"""Compute control-period evidence from record_cmd_vel.py output."""

import argparse
import csv
import math

import yaml


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--expected-period', type=float, default=0.05)
    args = parser.parse_args()

    stamps = []
    velocities = []
    with open(args.input, newline='') as stream:
        for row in csv.DictReader(stream):
            try:
                stamp = float(row['sim_stamp_s'])
                vx = float(row['linear_x_mps'])
                wz = float(row['angular_z_rps'])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(stamp):
                stamps.append(stamp)
                velocities.append((vx, wz))

    intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    p50 = percentile(intervals, 0.50)
    p95 = percentile(intervals, 0.95)
    maximum = max(intervals) if intervals else None
    mean = sum(intervals) / len(intervals) if intervals else None
    result = {
        'topic': '/cmd_vel',
        'samples': len(stamps),
        'interval_samples': len(intervals),
        'sim_duration_s': (stamps[-1] - stamps[0]) if len(stamps) > 1 else None,
        'expected_period_s': args.expected_period,
        'mean_period_s': mean,
        'median_period_s': p50,
        'p95_period_s': p95,
        'max_period_s': maximum,
        'mean_frequency_hz': (1.0 / mean) if mean and mean > 0 else None,
        'median_frequency_hz': (1.0 / p50) if p50 and p50 > 0 else None,
        'p95_frequency_hz': (1.0 / p95) if p95 and p95 > 0 else None,
        'p95_period_ratio_to_expected': (p95 / args.expected_period)
        if p95 is not None and args.expected_period > 0 else None,
        'nonzero_command_samples': sum(
            abs(vx) > 1e-6 or abs(wz) > 1e-6 for vx, wz in velocities),
        'status': 'PASS' if p95 is not None else 'INSUFFICIENT_SAMPLES',
    }
    with open(args.output, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(result, stream, sort_keys=False)


if __name__ == '__main__':
    main()
