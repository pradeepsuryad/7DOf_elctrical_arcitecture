"""
HIL trajectory replay runner for the 7-DoF arm.

Connects to the arm over a USB-CAN adapter, replays a trajectory CSV at the
correct rate, collects feedback from every joint, and writes a timestamped log.

Supports a --dry-run mode (no hardware) that echoes commands back as synthetic
feedback so you can verify the script and log format without an arm present.

Usage:
  # real hardware (CANable / PCAN-USB / Kvaser):
  python hil_runner.py                              \\
      --traj  trajectories/sine.csv                 \\
      --iface slcan  --channel COM3  --bitrate 500000

  # dry run (no hardware):
  python hil_runner.py --traj trajectories/sine.csv --dry-run

Dependencies:  pip install python-can
"""

import argparse
import csv
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# python-can is only imported when not in dry-run mode.
# Import attempted at startup so a missing install fails fast.
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False

from can_protocol import (
    NUM_JOINTS, JointFault,
    JointCommand, JointFeedback,
    cmd_id, decode_feedback,
    encode_command,
    FEEDBACK_CSV_HEADER, feedback_to_csv_row,
)


# ── Safety thresholds ─────────────────────────────────────────────────────────
# These are checked every tick. If any joint breaches them the runner
# immediately sends an idle command to all joints and exits.

MAX_POSITION_RAD   = 3.0     # software position safety fence (< π for margin)
MAX_VELOCITY_RADS  = 8.0     # rad/s — above this something has gone wrong
MAX_TEMP_C         = 80.0    # °C — motor winding limit
FEEDBACK_TIMEOUT_S = 0.1     # if a joint stops replying for 100 ms, abort


# ── Timing ────────────────────────────────────────────────────────────────────
# Python's scheduler has ~1 ms granularity on Windows and ~100 µs on Linux.
# We sleep until 600 µs before the deadline, then busy-wait the rest.
# This trades one CPU core for timing accuracy — acceptable for HIL sessions.

BUSYWAIT_THRESHOLD_S = 0.0006   # 600 µs


def _wait_until(deadline: float) -> None:
    remaining = deadline - time.perf_counter()
    if remaining > BUSYWAIT_THRESHOLD_S:
        time.sleep(remaining - BUSYWAIT_THRESHOLD_S)
    while time.perf_counter() < deadline:
        pass


# ── Dry-run fake bus ──────────────────────────────────────────────────────────

class _DryRunBus:
    """
    Simulates the arm by echoing commands back as synthetic feedback.
    Position moves toward the setpoint at 1 rad/s (first-order lag).
    Used to test the runner and log pipeline without real hardware.
    """
    def __init__(self, dt: float):
        self._pos  = [0.0] * NUM_JOINTS
        self._vel  = [0.0] * NUM_JOINTS
        self._dt   = dt

    def send(self, msg) -> None:
        # msg is a can.Message; decode to extract joint command.
        jid = msg.arbitration_id - 0x100
        if not (0 <= jid < NUM_JOINTS):
            return
        import struct
        setpoint, = struct.unpack_from('<f', bytes(msg.data), 0)
        # First-order lag toward setpoint (tau = 0.2 s).
        tau = 0.2
        alpha = self._dt / (tau + self._dt)
        self._pos[jid] += alpha * (setpoint - self._pos[jid])
        self._vel[jid]  = (setpoint - self._pos[jid]) / tau

    def recv(self, timeout: float):
        """Yield one synthetic feedback frame per joint."""
        import struct, can as _can
        frames = []
        for jid in range(NUM_JOINTS):
            data = bytearray(8)
            pos_raw    = int(round(self._pos[jid] * 1000.0))
            vel_raw    = int(round(self._vel[jid] * 100.0))
            data[0:2]  = struct.pack('<h', max(-32768, min(32767, pos_raw)))
            data[2:4]  = struct.pack('<h', max(-32768, min(32767, vel_raw)))
            data[4:6]  = b'\x00\x00'   # torque = 0
            data[6]    = 0             # no faults
            data[7]    = 30            # 30 °C
            frames.append(_can.Message(
                arbitration_id=0x300 + jid,
                data=data, is_extended_id=False,
                timestamp=time.time()))
        return frames   # caller iterates

    def shutdown(self) -> None:
        pass


# ── Trajectory loading ────────────────────────────────────────────────────────

@dataclass
class TrajectoryRow:
    time_s: float
    positions: list   # length NUM_JOINTS


def load_trajectory(path: Path) -> list[TrajectoryRow]:
    rows = []
    with path.open(newline='') as f:
        reader = csv.DictReader(f)
        for line in reader:
            positions = [float(line.get(f'j{i}', 0.0)) for i in range(NUM_JOINTS)]
            rows.append(TrajectoryRow(
                time_s=float(line['time_s']),
                positions=positions))
    if not rows:
        sys.exit(f"Trajectory file {path} is empty.")
    return rows


# ── Safety monitor ────────────────────────────────────────────────────────────

@dataclass
class SafetyMonitor:
    last_seen: list = field(default_factory=lambda: [0.0] * NUM_JOINTS)

    def check(self, fb: JointFeedback, now: float) -> Optional[str]:
        self.last_seen[fb.joint_id] = now

        if fb.faults != JointFault.NONE:
            return f"Joint {fb.joint_id} fault: {fb.faults!r}"
        if abs(fb.position) > MAX_POSITION_RAD:
            return f"Joint {fb.joint_id} position {fb.position:.3f} rad exceeds ±{MAX_POSITION_RAD} rad"
        if abs(fb.velocity) > MAX_VELOCITY_RADS:
            return f"Joint {fb.joint_id} velocity {fb.velocity:.2f} rad/s exceeds {MAX_VELOCITY_RADS}"
        if fb.temp_c > MAX_TEMP_C:
            return f"Joint {fb.joint_id} temperature {fb.temp_c:.0f} °C exceeds {MAX_TEMP_C} °C"
        return None

    def check_timeouts(self, now: float) -> Optional[str]:
        for jid in range(NUM_JOINTS):
            if self.last_seen[jid] > 0 and (now - self.last_seen[jid]) > FEEDBACK_TIMEOUT_S:
                return f"Joint {jid} feedback timeout ({now - self.last_seen[jid]:.3f} s)"
        return None


# ── Main runner ───────────────────────────────────────────────────────────────

def run(args: argparse.Namespace) -> int:
    traj_path = Path(args.traj)
    if not traj_path.exists():
        sys.exit(f"Trajectory not found: {traj_path}")

    trajectory = load_trajectory(traj_path)
    dt = trajectory[1].time_s - trajectory[0].time_s if len(trajectory) > 1 else 0.001
    print(f"Loaded {len(trajectory)} rows  dt={dt*1000:.2f} ms  "
          f"duration={trajectory[-1].time_s:.2f} s")

    # ── Open CAN bus ──────────────────────────────────────────────────────────
    if args.dry_run:
        bus = _DryRunBus(dt)
        print("Dry-run mode: synthetic feedback, no hardware required.")
    else:
        if not CAN_AVAILABLE:
            sys.exit("python-can not installed. Run: pip install python-can")
        bus = can.interface.Bus(
            interface=args.iface,
            channel=args.channel,
            bitrate=args.bitrate)
        print(f"Connected: {args.iface}  {args.channel}  {args.bitrate} bit/s")

    # ── Open log file ─────────────────────────────────────────────────────────
    repo_root = Path(__file__).resolve().parents[2]
    log_dir   = repo_root / 'hil' / 'logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp     = time.strftime('%Y%m%d_%H%M%S')
    log_path  = log_dir / f'hil_{traj_path.stem}_{stamp}.csv'

    safety   = SafetyMonitor()
    abort_reason: Optional[str] = None

    with log_path.open('w', newline='') as log_f:
        log_f.write(FEEDBACK_CSV_HEADER + '\n')

        start_wall = time.perf_counter()
        print(f"Replay started → log: {log_path.name}")
        print("Press Ctrl-C to abort.")

        try:
            for row_idx, row in enumerate(trajectory):
                tick_deadline = start_wall + row.time_s

                # ── Send command frame for each joint ─────────────────────────
                for jid in range(NUM_JOINTS):
                    cmd = JointCommand(
                        joint_id   = jid,
                        position   = row.positions[jid],
                        vel_limit  = args.vel_limit,
                        torque_cmd = 0.0,    # position-mode: torque set by PID
                    )
                    payload = encode_command(cmd)
                    msg = can.Message(
                        arbitration_id=cmd_id(jid),
                        data=payload,
                        is_extended_id=False)
                    bus.send(msg)

                # ── Collect feedback ──────────────────────────────────────────
                now = time.perf_counter()
                elapsed = now - start_wall

                if args.dry_run:
                    frames = bus.recv(timeout=0)
                else:
                    frames = []
                    # Drain whatever arrived since last tick.
                    while True:
                        msg = bus.recv(timeout=0)
                        if msg is None:
                            break
                        frames.append(msg)

                for msg in frames:
                    fb = decode_feedback(
                        msg.arbitration_id,
                        bytes(msg.data),
                        timestamp_s=elapsed)
                    if fb is None:
                        continue

                    reason = safety.check(fb, now)
                    if reason:
                        abort_reason = reason
                        break

                    log_f.write(feedback_to_csv_row(fb) + '\n')

                if abort_reason:
                    break

                # Timeout check every 100 ticks (every ~100 ms at 1 kHz).
                if row_idx % 100 == 0:
                    reason = safety.check_timeouts(now)
                    if reason:
                        abort_reason = reason
                        break

                # ── Progress print every 500 rows ─────────────────────────────
                if row_idx % 500 == 0:
                    pct = 100.0 * row_idx / len(trajectory)
                    print(f"  {pct:5.1f}%  t={row.time_s:.3f} s", end='\r')

                # ── Timing: wait for next tick deadline ───────────────────────
                _wait_until(tick_deadline)

        except KeyboardInterrupt:
            abort_reason = "operator Ctrl-C"

        # ── Idle all joints on exit ───────────────────────────────────────────
        print("\nSending idle commands to all joints...")
        for jid in range(NUM_JOINTS):
            cmd = JointCommand(joint_id=jid, position=0.0,
                               vel_limit=0.0, torque_cmd=0.0)
            msg = can.Message(
                arbitration_id=cmd_id(jid),
                data=encode_command(cmd),
                is_extended_id=False)
            bus.send(msg)

        bus.shutdown()

    if abort_reason:
        print(f"\nAborted: {abort_reason}")
        return 1

    print(f"\nDone. Log saved to: {log_path}")
    return 0


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay a trajectory CSV on the 7-DoF arm over CAN.")

    parser.add_argument('--traj',     required=True,
                        help='path to trajectory CSV')
    parser.add_argument('--iface',    default='slcan',
                        help='python-can interface (slcan, pcan, kvaser, ...)')
    parser.add_argument('--channel',  default='COM3',
                        help='CAN channel (COM port for slcan, PCAN_USBBUS1, etc.)')
    parser.add_argument('--bitrate',  type=int, default=500_000,
                        help='CAN bitrate (default 500000)')
    parser.add_argument('--vel-limit', type=float, default=4.0,
                        help='velocity limit sent in every command frame (rad/s)')
    parser.add_argument('--dry-run',  action='store_true',
                        help='simulate feedback without real hardware')

    args = parser.parse_args()
    sys.exit(run(args))


if __name__ == '__main__':
    main()
