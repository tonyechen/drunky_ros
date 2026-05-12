#!/usr/bin/env python3
"""Measure per-joint calibration offsets between ROS and lerobot for the
SO-101 follower, and emit the JOINT_OFFSETS_DEG table ready to paste into
soa_bringup/policy_calibration.py.

Usage (multi-pose, recommended):

    # 1. With soa_bringup running, hold arm still in some pose. Read ROS:
    python3 calibrate_policy_offsets.py ros

    # 2. Ctrl+C soa_bringup (so /dev/ttyACM3 is free). Arm servos hold.
    #    DO NOT MOVE THE ARM. Read lerobot:
    python3 calibrate_policy_offsets.py lerobot

    # 3. Restart soa_bringup, move to a different pose, repeat from step 1.
    #    Use 3-5 poses spread across the workspace.

    # 4. When you have several pose pairs, compute the table:
    python3 calibrate_policy_offsets.py compute

    # Reset and start over:
    python3 calibrate_policy_offsets.py reset

Samples accumulate in /tmp/calib_samples.json across runs. `compute` pairs
ROS sample i with lerobot sample i (so the ros / lerobot pair must come
from the same physical pose, in the order you captured them).

If the per-joint Δ is consistent across poses (std < ~1°), the mismatch
is a constant offset and the printed table is correct. If a joint's Δ
varies significantly with pose, that joint also has a sign-flip or scale
mismatch — pure offset won't fix it; you'll need to investigate the
calibration JSON application in feetech_ros2_driver / xacro.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

SAMPLES_PATH = Path("/tmp/calib_samples.json")

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
              "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
ALL_JOINTS = ARM_JOINTS + [GRIPPER]

# Hardware identity (matches launch_commands.md / soa_params.yaml).
PORT = "/dev/ttyACM3"
ROBOT_ID = "gix-follower4"
CALIB_DIR = Path(
    "/home/ubuntu/techin517/huggingface/lerobot/calibration/robots/so101_follower"
)


# ---------- shared sample I/O ----------

def _load_samples() -> dict:
    if SAMPLES_PATH.exists():
        return json.loads(SAMPLES_PATH.read_text())
    return {"ros": [], "lerobot": []}


def _save_samples(data: dict) -> None:
    SAMPLES_PATH.write_text(json.dumps(data, indent=2))


def _fmt_deg_row(values: dict[str, float]) -> str:
    return "  ".join(f"{j:>13s}={values[j]:+7.2f}" for j in ALL_JOINTS)


# ---------- mode: ros ----------

def mode_ros(duration_s: float) -> int:
    """Average /follower/joint_states for `duration_s` seconds and append the
    pose-mean (in degrees) to the samples file under "ros"."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    class Reader(Node):
        def __init__(self):
            super().__init__("calib_ros_reader")
            self.samples: list[dict[str, float]] = []
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
                self.samples.append({j: d[j] for j in ALL_JOINTS})

    rclpy.init()
    node = Reader()
    t0 = time.time()
    print(f"Sampling /follower/joint_states for {duration_s:.1f}s... "
          "(hold the arm still)")
    while time.time() - t0 < duration_s:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()

    if not node.samples:
        print("ERROR: no /follower/joint_states messages received. "
              "Is soa_bringup running?")
        return 1

    n = len(node.samples)
    mean_deg = {
        j: math.degrees(sum(s[j] for s in node.samples) / n)
        for j in ALL_JOINTS
    }
    print(f"Captured {n} samples. Mean pose (deg):")
    print("  " + _fmt_deg_row(mean_deg))

    data = _load_samples()
    data["ros"].append(mean_deg)
    _save_samples(data)
    pair_idx = len(data["ros"])
    print(f"Stored as ROS sample #{pair_idx}. Now stop soa_bringup and run:")
    print(f"  python3 {sys.argv[0]} lerobot")
    return 0


# ---------- mode: lerobot ----------

def mode_lerobot(n_samples: int) -> int:
    """Connect via lerobot.SO101Follower, average n readings, append to file."""
    try:
        from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
    except ImportError as e:
        print(f"ERROR: cannot import lerobot.SO101Follower: {e}")
        return 1

    cfg = SO101FollowerConfig(
        port=PORT, id=ROBOT_ID, calibration_dir=CALIB_DIR,
    )
    robot = SO101Follower(cfg)
    print(f"Connecting via lerobot at {PORT}...")
    try:
        robot.connect(calibrate=False)
    except Exception as e:
        print(f"ERROR connecting: {e}")
        print(f"Is soa_bringup stopped? {PORT} must be free.")
        return 1

    try:
        samples = []
        for _ in range(n_samples):
            samples.append(robot.get_observation())
            time.sleep(0.05)
    finally:
        robot.disconnect()

    n = len(samples)
    mean_deg = {}
    for j in ALL_JOINTS:
        key = f"{j}.pos"
        if key not in samples[0]:
            print(f"ERROR: lerobot observation missing '{key}'. "
                  f"Got: {sorted(samples[0])}")
            return 1
        mean_deg[j] = sum(s[key] for s in samples) / n

    print(f"Captured {n} samples. Mean pose (deg):")
    print("  " + _fmt_deg_row(mean_deg))

    data = _load_samples()
    data["lerobot"].append(mean_deg)
    _save_samples(data)
    pair_idx = len(data["lerobot"])
    n_ros = len(data["ros"])
    print(f"Stored as lerobot sample #{pair_idx} (ros samples: {n_ros}).")
    if pair_idx < n_ros:
        print("You have more ROS samples than lerobot samples — "
              "the next lerobot capture will pair with the next ROS one.")
    elif pair_idx > n_ros:
        print("WARNING: you have more lerobot samples than ROS samples. "
              "Pairing assumes same index → same pose.")
    return 0


# ---------- mode: compute ----------

def mode_compute() -> int:
    """Pair ROS sample i with lerobot sample i and emit per-joint offset stats."""
    data = _load_samples()
    n_pairs = min(len(data["ros"]), len(data["lerobot"]))
    if n_pairs == 0:
        print("No paired samples. Capture at least one (ros + lerobot) first.")
        return 1
    if len(data["ros"]) != len(data["lerobot"]):
        print(f"WARNING: {len(data['ros'])} ROS samples vs "
              f"{len(data['lerobot'])} lerobot samples — using first "
              f"{n_pairs} pairs.")

    print(f"\nUsing {n_pairs} pose pair(s).\n")

    # Δ = lerobot_deg - ros_deg, per joint per pose.
    deltas: dict[str, list[float]] = {j: [] for j in ALL_JOINTS}
    for i in range(n_pairs):
        ros_p = data["ros"][i]
        lr_p = data["lerobot"][i]
        for j in ALL_JOINTS:
            deltas[j].append(lr_p[j] - ros_p[j])

    print(f"{'joint':<14s}  {'Δ mean':>8s}  {'Δ std':>7s}  {'Δ range':>16s}"
          f"  {'verdict':<28s}")
    print(f"{'-'*14}  {'-'*8}  {'-'*7}  {'-'*16}  {'-'*28}")
    means: dict[str, float] = {}
    for j in ALL_JOINTS:
        vals = deltas[j]
        m = statistics.fmean(vals)
        means[j] = m
        s = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        lo, hi = min(vals), max(vals)

        if n_pairs == 1:
            verdict = "(need more poses to verify)"
        elif s < 0.5:
            verdict = "constant offset ✓"
        elif s < 2.0:
            verdict = "noisy but ~constant"
        else:
            verdict = "NONLINEAR — pure offset won't fix"

        print(f"{j:<14s}  {m:>+8.2f}  {s:>7.2f}  [{lo:+6.2f}, {hi:+6.2f}]"
              f"  {verdict}")

    print("\nPaste this into "
          "ros2_ws/src/soa_ros2/soa_bringup/soa_bringup/policy_calibration.py:")
    print()
    print("JOINT_OFFSETS_DEG: dict[str, float] = {")
    for j in ALL_JOINTS:
        # Two-space alignment to match the existing dict style.
        print(f'    "{j}":  {means[j]:+6.2f},')
    print("}")
    print()
    return 0


# ---------- mode: reset ----------

def mode_reset() -> int:
    if SAMPLES_PATH.exists():
        SAMPLES_PATH.unlink()
        print(f"Deleted {SAMPLES_PATH}.")
    else:
        print("No samples file to delete.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["ros", "lerobot", "compute", "reset", "status"])
    parser.add_argument("--ros-duration", type=float, default=2.0,
                        help="Seconds to average /follower/joint_states (default 2.0)")
    parser.add_argument("--lerobot-samples", type=int, default=10,
                        help="Number of lerobot reads to average (default 10)")
    args = parser.parse_args()

    if args.mode == "ros":
        return mode_ros(args.ros_duration)
    if args.mode == "lerobot":
        return mode_lerobot(args.lerobot_samples)
    if args.mode == "compute":
        return mode_compute()
    if args.mode == "reset":
        return mode_reset()
    if args.mode == "status":
        data = _load_samples()
        print(f"ROS samples:     {len(data['ros'])}")
        print(f"lerobot samples: {len(data['lerobot'])}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
