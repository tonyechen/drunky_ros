#!/usr/bin/env python3
"""Single-session interactive calibration of the ROS↔lerobot per-joint offset
for the SO-101 follower. Mimics the spirit of `lerobot`'s calibration UX:
move the arm to a pose, tap ENTER, capture; repeat.

Constraint: ROS (`ros2_control_node`) and lerobot (`SO101Follower`) both
need exclusive access to /dev/ttyACM3. We can't read both stacks
simultaneously while the arm moves. So the procedure is:

  PHASE 1 (soa_bringup must be running):
    For each pose you want to sample, hold the arm still and press ENTER.
    The script averages /follower/joint_states for ~1 s and stores it.

  PHASE 2 (you stop soa_bringup mid-script):
    Same N poses, same order. Servos hold their last position when ROS
    disconnects, so you should be able to reproduce poses from muscle
    memory — but you need to physically NOT bump the arm between phases.
    Best practice: visually mark poses with landmarks (bottle positions,
    a sticker on the table, etc.) so phase-2 reproduction is reliable.

The script then pairs sample i in phase 1 with sample i in phase 2,
computes lerobot_deg - ros_deg per joint, and reports:

  - mean offset (the number to paste into JOINT_OFFSETS_DEG)
  - std across poses (low std → constant offset, high std → scale or
    sign issue, pure offset won't fix)
  - optional linear fit: delta(j) = a * ros_value(j) + b
    (use this if std is high; tells you if there's a scale mismatch too)

Usage:
    # Single session, prompts will tell you what to do:
    python3 calibrate_interactive.py

    # If you've already captured pose data and just want to re-compute:
    python3 calibrate_interactive.py --replay /tmp/calib_session.json

    # Add --linear-fit to also fit y = a*x + b per joint and report `a`.
    python3 calibrate_interactive.py --linear-fit
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

PORT = "/dev/ttyACM3"
ROBOT_ID = "gix-follower4"
CALIB_DIR = Path(
    "/home/ubuntu/techin517/huggingface/lerobot/calibration/robots/so101_follower"
)

SESSION_PATH = Path("/tmp/calib_session.json")


def _fmt_pose(p: dict[str, float]) -> str:
    return "  ".join(f"{j[:9]:>9s}={p[j]:+7.2f}" for j in ALL_JOINTS)


# -------------------------------------------------------------------
# Phase 1: ROS — average /follower/joint_states for ~1 s per pose
# -------------------------------------------------------------------

def capture_ros_poses() -> list[dict[str, float]]:
    """Interactive: prompt user for poses, capture each via rclpy spin."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    print("\n" + "=" * 70)
    print("PHASE 1 — ROS readings")
    print("=" * 70)
    print("Make sure soa_bringup is running. The script will average")
    print("/follower/joint_states for ~1 s at each pose.")
    print()
    print("For each pose:")
    print("  • Move the arm to a stable, repeatable pose")
    print("  • Wait for it to settle")
    print("  • Press ENTER to capture")
    print("  • Type 'q' + ENTER when done (recommend 3-5 poses)")
    print()
    print("Choose poses that are visually distinguishable (different bottle")
    print("positions, different reach distances) — you'll need to reproduce")
    print("them in Phase 2.")
    print()

    class Reader(Node):
        def __init__(self):
            super().__init__("calib_ros_reader")
            self.samples: list[dict[str, float]] = []
            self.latest: dict[str, float] | None = None
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
            )
            self.create_subscription(
                JointState, "/follower/joint_states", self._cb, qos
            )

        def _cb(self, msg):
            d = {n: p for n, p in zip(msg.name, msg.position)}
            if all(j in d for j in ALL_JOINTS):
                self.latest = {j: d[j] for j in ALL_JOINTS}

        def collect(self, duration_s: float) -> dict[str, float]:
            self.samples = []
            t0 = time.time()
            while time.time() - t0 < duration_s:
                if self.latest is not None:
                    self.samples.append(self.latest)
                rclpy.spin_once(self, timeout_sec=0.05)
            if not self.samples:
                raise RuntimeError(
                    "No /follower/joint_states messages received. "
                    "Is soa_bringup running?"
                )
            n = len(self.samples)
            return {
                j: math.degrees(sum(s[j] for s in self.samples) / n)
                for j in ALL_JOINTS
            }

    rclpy.init()
    node = Reader()
    # Spin briefly to populate self.latest before first capture.
    t0 = time.time()
    while node.latest is None and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        rclpy.shutdown()
        raise RuntimeError(
            "Could not connect to /follower/joint_states. Is soa_bringup running?"
        )

    poses: list[dict[str, float]] = []
    while True:
        prompt = f"  Pose {len(poses) + 1}: ENTER to capture, 'q'+ENTER to finish > "
        try:
            line = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if line == "q":
            break
        try:
            pose = node.collect(duration_s=1.0)
        except RuntimeError as e:
            print(f"    {e}")
            continue
        poses.append(pose)
        print(f"    captured: {_fmt_pose(pose)}")

    rclpy.shutdown()
    return poses


# -------------------------------------------------------------------
# Phase 2: lerobot — connect via SO101Follower, capture matching poses
# -------------------------------------------------------------------

def capture_lerobot_poses(n_expected: int) -> list[dict[str, float]]:
    """Wait for soa_bringup to be stopped, connect via lerobot, capture poses."""
    print("\n" + "=" * 70)
    print("PHASE 2 — lerobot readings")
    print("=" * 70)
    print(f"Stop soa_bringup so {PORT} is free. The servos will retain their")
    print("last position when ros2_control_node disconnects.")
    print()
    input(f"Press ENTER when soa_bringup is stopped > ")

    try:
        from lerobot.robots.so101_follower import (
            SO101Follower, SO101FollowerConfig
        )
    except ImportError as e:
        raise RuntimeError(f"Cannot import lerobot.SO101Follower: {e}")

    cfg = SO101FollowerConfig(port=PORT, id=ROBOT_ID, calibration_dir=CALIB_DIR)
    robot = SO101Follower(cfg)
    try:
        robot.connect(calibrate=False)
    except Exception as e:
        raise RuntimeError(
            f"Failed to connect via lerobot at {PORT}: {e}\n"
            "Is soa_bringup fully stopped? Any leftover ros2_control_node holds the port."
        )

    poses: list[dict[str, float]] = []
    try:
        print()
        print(f"Now move the arm to each pose in the SAME ORDER as Phase 1.")
        print(f"You captured {n_expected} pose(s) in Phase 1.")
        print()
        while len(poses) < n_expected:
            i = len(poses)
            try:
                input(f"  Pose {i + 1}/{n_expected}: position arm "
                      f"(same as Phase 1 pose {i + 1}) and press ENTER > ")
            except (EOFError, KeyboardInterrupt):
                print()
                print(f"  Aborted. Got {len(poses)} of {n_expected} lerobot poses.")
                break

            # Average a handful of reads.
            samples = []
            for _ in range(15):
                samples.append(robot.get_observation())
                time.sleep(0.03)

            n = len(samples)
            pose = {}
            ok = True
            for j in ALL_JOINTS:
                key = f"{j}.pos"
                if key not in samples[0]:
                    print(f"    ERROR: lerobot observation missing '{key}'")
                    ok = False
                    break
                pose[j] = sum(s[key] for s in samples) / n
            if not ok:
                continue
            poses.append(pose)
            print(f"    captured: {_fmt_pose(pose)}")
    finally:
        try:
            robot.disconnect()
        except Exception:
            pass

    return poses


# -------------------------------------------------------------------
# Analysis: pair samples, compute deltas, fit linear model
# -------------------------------------------------------------------

def analyse(ros_poses: list[dict[str, float]],
            lr_poses: list[dict[str, float]],
            linear_fit: bool) -> None:
    n = min(len(ros_poses), len(lr_poses))
    if n == 0:
        print("\nNo paired poses to analyse.")
        return
    if len(ros_poses) != len(lr_poses):
        print(f"\nWARNING: {len(ros_poses)} ROS poses vs {len(lr_poses)} "
              f"lerobot poses — using first {n} pairs.")

    print("\n" + "=" * 70)
    print(f"RESULTS — {n} pose pair(s)")
    print("=" * 70)

    # Per-joint delta lists, paired with the ROS reading for fitting.
    deltas: dict[str, list[float]] = {j: [] for j in ALL_JOINTS}
    ros_vals: dict[str, list[float]] = {j: [] for j in ALL_JOINTS}
    for i in range(n):
        for j in ALL_JOINTS:
            deltas[j].append(lr_poses[i][j] - ros_poses[i][j])
            ros_vals[j].append(ros_poses[i][j])

    print(f"\n{'joint':<14s}  {'Δ mean':>8s}  {'Δ std':>7s}  {'Δ range':>18s}"
          f"  {'verdict':<30s}")
    print(f"{'-'*14}  {'-'*8}  {'-'*7}  {'-'*18}  {'-'*30}")
    means: dict[str, float] = {}
    for j in ALL_JOINTS:
        vals = deltas[j]
        m = statistics.fmean(vals)
        means[j] = m
        s = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        lo, hi = min(vals), max(vals)
        if n == 1:
            verdict = "1 pose — can't verify"
        elif s < 0.5:
            verdict = "constant offset ✓"
        elif s < 2.0:
            verdict = "noisy but ~constant"
        else:
            verdict = "NONLINEAR — see linear fit"
        print(f"{j:<14s}  {m:>+8.2f}  {s:>7.2f}  [{lo:+7.2f}, {hi:+7.2f}]"
              f"  {verdict}")

    # Optional linear fit per joint: delta = a * ros + b. If `a` is far
    # from 0 and the residual std drops a lot, the joint has a scale
    # mismatch in addition to (or instead of) a pure offset.
    if linear_fit and n >= 2:
        print(f"\nLinear fit per joint (Δ = a · ros_reading + b):")
        print(f"{'joint':<14s}  {'a (slope)':>10s}  {'b (intercept)':>14s}  "
              f"{'resid std':>9s}  {'note':<35s}")
        print(f"{'-'*14}  {'-'*10}  {'-'*14}  {'-'*9}  {'-'*35}")
        for j in ALL_JOINTS:
            xs = ros_vals[j]
            ys = deltas[j]
            xm = statistics.fmean(xs)
            ym = statistics.fmean(ys)
            num = sum((xs[i] - xm) * (ys[i] - ym) for i in range(n))
            den = sum((xs[i] - xm) ** 2 for i in range(n))
            a = num / den if den > 1e-9 else 0.0
            b = ym - a * xm
            resid = [ys[i] - (a * xs[i] + b) for i in range(n)]
            r_std = statistics.pstdev(resid) if n > 1 else 0.0
            if abs(a) < 0.02:
                note = "pure offset"
            elif r_std < 0.5:
                note = "scale + offset — replace with linear"
            else:
                note = "model doesn't fit well"
            print(f"{j:<14s}  {a:>+10.4f}  {b:>+14.2f}  {r_std:>9.2f}  {note:<35s}")

    print("\nPaste this into "
          "ros2_ws/src/soa_ros2/soa_bringup/soa_bringup/policy_calibration.py:\n")
    print("JOINT_OFFSETS_DEG: dict[str, float] = {")
    for j in ALL_JOINTS:
        print(f'    "{j}":  {means[j]:+6.2f},')
    print("}\n")


# -------------------------------------------------------------------
# Session persistence
# -------------------------------------------------------------------

def save_session(ros_poses: list[dict[str, float]],
                 lr_poses: list[dict[str, float]]) -> None:
    SESSION_PATH.write_text(json.dumps(
        {"ros": ros_poses, "lerobot": lr_poses}, indent=2
    ))
    print(f"Session saved to {SESSION_PATH}")


def load_session(path: Path) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    d = json.loads(path.read_text())
    return d["ros"], d["lerobot"]


# -------------------------------------------------------------------
# main
# -------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--replay", type=Path, default=None,
        help="Skip capture; re-analyse a previous session JSON.",
    )
    parser.add_argument(
        "--linear-fit", action="store_true",
        help="Also fit Δ = a·ros + b per joint (use if some joints have high Δ std).",
    )
    args = parser.parse_args()

    if args.replay is not None:
        if not args.replay.exists():
            print(f"ERROR: {args.replay} does not exist.")
            return 1
        ros_poses, lr_poses = load_session(args.replay)
        analyse(ros_poses, lr_poses, linear_fit=args.linear_fit)
        return 0

    try:
        ros_poses = capture_ros_poses()
    except RuntimeError as e:
        print(f"ERROR in Phase 1: {e}")
        return 1
    if not ros_poses:
        print("No poses captured in Phase 1. Exiting.")
        return 1

    try:
        lr_poses = capture_lerobot_poses(n_expected=len(ros_poses))
    except RuntimeError as e:
        print(f"ERROR in Phase 2: {e}")
        # Still save what we have.
        save_session(ros_poses, [])
        return 1

    save_session(ros_poses, lr_poses)
    analyse(ros_poses, lr_poses, linear_fit=args.linear_fit)
    return 0


if __name__ == "__main__":
    sys.exit(main())
