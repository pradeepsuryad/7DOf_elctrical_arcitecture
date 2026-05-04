"""
Generate test trajectories for HIL replay.

Outputs a CSV with columns:
  time_s, j0, j1, j2, j3, j4, j5, j6   (positions in radians)

Usage:
  python gen_trajectory.py sine   --duration 5 --amplitude 0.5 --hz 0.5
  python gen_trajectory.py step   --duration 3 --amplitude 0.3 --joints 0,1
  python gen_trajectory.py ramp   --duration 4 --target 1.0    --joints 0
  python gen_trajectory.py zeros  --duration 2                 # safe home run

All joints not listed in --joints stay at 0 rad throughout.

Why these shapes?
  sine  — smooth, bounded — good first test; reveals steady-state tracking error
  step  — instantaneous demand — reveals overshoot, ringing, PID tuning problems
  ramp  — constant velocity — reveals velocity-loop gain and friction compensation
  zeros — hold position at 0 for every joint — minimum viable sanity check
"""

import argparse
import csv
import math
import sys
from pathlib import Path


NUM_JOINTS = 7
DEFAULT_DT  = 0.001   # 1 kHz sample rate matches firmware inner loop


def _parse_joints(s: str) -> list[int]:
    """'0,1,3' → [0, 1, 3]; empty/None → all joints."""
    if not s:
        return list(range(NUM_JOINTS))
    ids = [int(x.strip()) for x in s.split(',')]
    for jid in ids:
        if not (0 <= jid < NUM_JOINTS):
            sys.exit(f"Joint ID {jid} is out of range (0–{NUM_JOINTS-1})")
    return ids


def _write_csv(path: Path, rows: list[list[float]]) -> None:
    with path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s'] + [f'j{i}' for i in range(NUM_JOINTS)])
        for row in rows:
            writer.writerow([f'{v:.6f}' for v in row])
    print(f"Wrote {len(rows)} rows → {path}")


# ── Trajectory generators ─────────────────────────────────────────────────────

def gen_sine(duration: float, amplitude: float, freq_hz: float,
             joints: list[int], dt: float) -> list[list[float]]:
    """
    Sinusoidal oscillation: pos(t) = amplitude × sin(2π × freq × t)

    Good first test — smooth, bounded, easy to visualise.
    Tracking error at steady state tells you how well the position loop is tuned.
    Phase lag between command and feedback reveals bandwidth limits.
    """
    rows = []
    t = 0.0
    n = int(duration / dt)
    for _ in range(n):
        pos = [0.0] * NUM_JOINTS
        val = amplitude * math.sin(2 * math.pi * freq_hz * t)
        for jid in joints:
            pos[jid] = val
        rows.append([t] + pos)
        t += dt
    return rows


def gen_step(duration: float, amplitude: float, step_time: float,
             joints: list[int], dt: float) -> list[list[float]]:
    """
    Step demand: 0 → amplitude at t = step_time, then holds.

    Worst case for PID overshoot and settling time.
    A well-tuned cascade should reach steady state in <500 ms with <5% overshoot.
    If the step response rings or diverges, reduce kp or increase kd.
    """
    rows = []
    t = 0.0
    n = int(duration / dt)
    for _ in range(n):
        pos = [0.0] * NUM_JOINTS
        val = amplitude if t >= step_time else 0.0
        for jid in joints:
            pos[jid] = val
        rows.append([t] + pos)
        t += dt
    return rows


def gen_ramp(duration: float, target: float,
             joints: list[int], dt: float) -> list[list[float]]:
    """
    Linear ramp from 0 to `target` over `duration` seconds.

    Velocity = target / duration (constant throughout).
    Used to verify the velocity loop gain and expose static friction.
    If actual position lags behind the ramp, increase the velocity-loop kp.
    """
    rows = []
    t = 0.0
    n = int(duration / dt)
    for i in range(n):
        pos = [0.0] * NUM_JOINTS
        val = target * (i / max(n - 1, 1))
        for jid in joints:
            pos[jid] = val
        rows.append([t] + pos)
        t += dt
    return rows


def gen_zeros(duration: float, dt: float) -> list[list[float]]:
    """Hold all joints at 0 rad. Minimum sanity check before any motion."""
    rows = []
    t = 0.0
    n = int(duration / dt)
    for _ in range(n):
        rows.append([t] + [0.0] * NUM_JOINTS)
        t += dt
    return rows


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Generate HIL test trajectories.")
    sub = parser.add_subparsers(dest='shape', required=True)

    # shared
    def add_common(p):
        p.add_argument('--duration', type=float, default=5.0, help='seconds')
        p.add_argument('--dt',       type=float, default=DEFAULT_DT,
                       help='sample period (s), default 0.001')
        p.add_argument('--out',      type=str,   default=None,
                       help='output CSV path (default: trajectories/<shape>.csv)')
        p.add_argument('--joints',   type=str,   default='',
                       help='comma-separated joint IDs, e.g. "0,1" (default: all)')

    p_sine = sub.add_parser('sine')
    add_common(p_sine)
    p_sine.add_argument('--amplitude', type=float, default=0.5, help='rad')
    p_sine.add_argument('--hz',        type=float, default=0.5, help='frequency')

    p_step = sub.add_parser('step')
    add_common(p_step)
    p_step.add_argument('--amplitude', type=float, default=0.3,  help='rad')
    p_step.add_argument('--step-time', type=float, default=0.5,  help='s')

    p_ramp = sub.add_parser('ramp')
    add_common(p_ramp)
    p_ramp.add_argument('--target', type=float, default=1.0, help='rad')

    p_zeros = sub.add_parser('zeros')
    add_common(p_zeros)

    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    traj_dir  = repo_root / 'hil' / 'trajectories'
    traj_dir.mkdir(parents=True, exist_ok=True)
    out_path  = Path(args.out) if args.out else traj_dir / f'{args.shape}.csv'
    joints    = _parse_joints(args.joints)
    dt        = args.dt

    if args.shape == 'sine':
        rows = gen_sine(args.duration, args.amplitude, args.hz, joints, dt)
    elif args.shape == 'step':
        rows = gen_step(args.duration, args.amplitude, args.step_time, joints, dt)
    elif args.shape == 'ramp':
        rows = gen_ramp(args.duration, args.target, joints, dt)
    else:  # zeros
        rows = gen_zeros(args.duration, dt)

    _write_csv(out_path, rows)


if __name__ == '__main__':
    main()
