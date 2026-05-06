#!/usr/bin/env python3
"""
Plot trajectory CSV files and (optionally) HIL log CSVs side-by-side.

Usage:
  # Just a trajectory command profile:
  python hil/scripts/plot_trajectory.py --traj hil/trajectories/sine.csv

  # Trajectory + actual HIL log overlaid (shows tracking error):
  python hil/scripts/plot_trajectory.py \
      --traj hil/trajectories/sine.csv \
      --log  hil/logs/hil_sine_<timestamp>.csv \
      --joints 0 1 2
"""
import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# ── CSV loaders ───────────────────────────────────────────────────────────────

def load_traj(path: str):
    """Load trajectory CSV -> (time_s array, positions dict {j0..j6})."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    t = np.array([r["time_s"] for r in rows])
    joints = {k: np.array([r[k] for r in rows])
              for k in rows[0] if k != "time_s"}
    return t, joints


def load_log(path: str):
    """Load HIL log CSV -> (time_s, actual positions dict)."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    if not rows:
        return None, None
    t = np.array([r["time_s"] for r in rows])
    joints = {}
    for k in rows[0]:
        if k.startswith("j") and "pos" in k:
            joints[k] = np.array([r[k] for r in rows])
    return t, joints


# ── Plotting ──────────────────────────────────────────────────────────────────

COLORS = plt.rcParams["axes.prop_cycle"].by_key()["color"]


def plot_trajectory_only(t, traj_joints, selected, title, out_path):
    n = len(selected)
    fig, axes = plt.subplots(n, 1, figsize=(12, 2.5 * n), sharex=True)
    if n == 1:
        axes = [axes]

    fig.suptitle(title, fontsize=14, fontweight="bold")

    for ax, jkey in zip(axes, selected):
        col = COLORS[int(jkey[1]) % len(COLORS)]
        ax.plot(t, np.degrees(traj_joints[jkey]), color=col, linewidth=1.8,
                label=f"Command {jkey}")
        ax.set_ylabel("Position (deg)", fontsize=9)
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.35)
        ax.axhline(0, color="black", linewidth=0.5, linestyle="--")

    axes[-1].set_xlabel("Time (s)", fontsize=10)
    plt.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved -> {out_path}")
    plt.show()


def plot_with_log(t_cmd, traj_joints, t_log, log_joints, selected, title, out_path):
    n = len(selected)
    fig = plt.figure(figsize=(13, 3 * n + 2))
    gs = gridspec.GridSpec(n + 1, 1, height_ratios=[3] * n + [1.5])

    fig.suptitle(title, fontsize=14, fontweight="bold")

    error_rms_all = []
    axes = []
    for i, jkey in enumerate(selected):
        ax = fig.add_subplot(gs[i], sharex=axes[0] if axes else None)
        axes.append(ax)

        col = COLORS[int(jkey[1]) % len(COLORS)]
        ax.plot(t_cmd, np.degrees(traj_joints[jkey]), color=col,
                linewidth=2, label="Command", zorder=3)

        # Try to find matching actual column (j0_pos, j0, etc.)
        actual_key = None
        for candidate in [f"{jkey}_pos", jkey, f"joint{jkey[1]}_pos"]:
            if candidate in log_joints:
                actual_key = candidate
                break

        if actual_key:
            ax.plot(t_log, np.degrees(log_joints[actual_key]),
                    color=col, linewidth=1.2, linestyle="--",
                    alpha=0.85, label="Actual")
            # Interpolate command onto log time for error
            cmd_interp = np.interp(t_log, t_cmd, traj_joints[jkey])
            err_deg = np.degrees(log_joints[actual_key] - cmd_interp)
            rms = np.sqrt(np.mean(err_deg ** 2))
            error_rms_all.append((jkey, rms))
            ax.fill_between(t_log,
                            np.degrees(log_joints[actual_key]),
                            np.degrees(cmd_interp),
                            alpha=0.15, color=col, label=f"Error (RMS {rms:.2f}°)")

        ax.set_ylabel("Position (deg)", fontsize=9)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.35)

    axes[-1].set_xlabel("Time (s)", fontsize=10)

    # RMS error bar chart
    if error_rms_all:
        ax_err = fig.add_subplot(gs[n])
        labels = [e[0] for e in error_rms_all]
        rms_vals = [e[1] for e in error_rms_all]
        bars = ax_err.bar(labels, rms_vals,
                          color=[COLORS[int(j[1]) % len(COLORS)] for j in labels])
        ax_err.set_ylabel("RMS error (deg)", fontsize=9)
        ax_err.set_title("Tracking Error per Joint", fontsize=10)
        ax_err.grid(True, axis="y", alpha=0.35)
        for bar, val in zip(bars, rms_vals):
            ax_err.text(bar.get_x() + bar.get_width() / 2, val + 0.01,
                        f"{val:.2f}", ha="center", va="bottom", fontsize=8)

    plt.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved -> {out_path}")
    plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Plot trajectory and/or HIL log")
    parser.add_argument("--traj",    required=True, help="Trajectory CSV path")
    parser.add_argument("--log",     default=None,  help="HIL log CSV (optional)")
    parser.add_argument("--joints",  nargs="+", type=int, default=list(range(7)),
                        help="Which joints to plot (0-6, default all)")
    parser.add_argument("--out",     default=None,
                        help="Output PNG path (default: auto-named in hil/logs/)")
    args = parser.parse_args()

    traj_path = Path(args.traj)
    if not traj_path.exists():
        sys.exit(f"Trajectory file not found: {traj_path}")

    t_cmd, traj_joints = load_traj(str(traj_path))
    available = [f"j{i}" for i in args.joints if f"j{i}" in traj_joints]
    if not available:
        sys.exit(f"None of the requested joints found in {traj_path}")

    traj_name = traj_path.stem
    out_dir = traj_path.parent.parent / "logs"
    out_dir.mkdir(exist_ok=True)

    if args.log:
        log_path = Path(args.log)
        if not log_path.exists():
            sys.exit(f"Log file not found: {log_path}")
        t_log, log_joints = load_log(str(log_path))
        title = f"HIL Tracking — {traj_name}  |  joints {args.joints}"
        out_path = args.out or str(out_dir / f"plot_{traj_name}_tracking.png")
        plot_with_log(t_cmd, traj_joints, t_log, log_joints,
                      available, title, out_path)
    else:
        title = f"Trajectory Command Profile — {traj_name}  |  joints {args.joints}"
        out_path = args.out or str(out_dir / f"plot_{traj_name}_cmd.png")
        plot_trajectory_only(t_cmd, traj_joints, available, title, out_path)


if __name__ == "__main__":
    main()
