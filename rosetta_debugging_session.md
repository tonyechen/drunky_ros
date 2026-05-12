# Rosetta + grab_and_pour_v5 ACT Policy — Debugging Session

Record of the investigation and fixes that took the grab_and_pour_right_v5 ACT
policy from "jitters in place under Rosetta" to "executes the full grab → pour
→ dispose trajectory."

---

## Initial Symptom

Running the grab_and_pour_v5 ACT policy through Rosetta caused the arm to
stay in one place and jitter. The model works correctly when run through
`lerobot-record`:

```bash
lerobot-record --robot.type=so101_follower --robot.port=/dev/ttyACM3 \
  --robot.id=gix-follower4 \
  --robot.cameras="{ arm: {type: opencv, index_or_path: '/dev/video0', ...}, \
                     middle: {type: opencv, index_or_path: '/dev/video8', ...}}" \
  --policy.path=outputs/train/grab_and_pour_right_v5/checkpoints/last/pretrained_model \
  --dataset.single_task="grab and pour"
```

But fails through Rosetta:

```bash
ros2 launch soa_bringup soa_bringup.launch.py
ros2 launch rosetta rosetta_client_launch.py \
  contract_path:=.../so_101_grab_and_pour.yaml \
  pretrained_name_or_path:=.../grab_and_pour_right_v5/checkpoints/last/pretrained_model \
  policy_type:=act
ros2 action send_goal /run_policy rosetta_interfaces/action/RunPolicy "{prompt: 'grab and pour'}"
```

---

## Stack Overview

| Component | Path | Role |
|---|---|---|
| Training data | `huggingface/lerobot/ubuntu/drunky_right/` | 66 episodes, 30 fps, 480×640 RGB frames |
| Trained policy | `outputs/train/grab_and_pour_right_v5/checkpoints/last/pretrained_model` | ACT, `n_action_steps=100`, `chunk_size=100`, `temporal_ensemble_coeff=null` |
| LeRobot install | `/home/ubuntu/.local/lib/python3.10/site-packages/lerobot` | v0.4.0 |
| Rosetta contract | `ros2_ws/src/rosetta/contracts/so_101_grab_and_pour.yaml` | Maps ROS topics ↔ policy I/O |
| Rosetta params | `ros2_ws/src/rosetta/params/rosetta_client.yaml` | Inference cadence knobs |
| ROS hardware bringup | `ros2_ws/src/soa_ros2/soa_bringup/` | feetech_ros2_driver + xacro-baked calibration |

---

## Root Causes Found (in order of discovery)

### 1. Observation-similarity filter silently dropping observations

LeRobot 0.4.0's `policy_server.py` runs every incoming observation through:

```python
# helpers.py:280
def observations_similar(obs1, obs2, lerobot_features, atol: float = 1) -> bool:
    return _compare_observation_states(obs1_state, obs2_state, atol=atol)
```

With `unit_conversion: rad2deg` in the contract, the state vector is in
**degrees**. The arm rarely moves ≥1° between two frames at 30 Hz, so the
server dropped nearly every observation and only re-inferred when
`must_go=True` fired (every ~1 s when the action queue drained).

Rosetta's YAML has an `obs_similarity_atol: -1.0` knob meant to disable this,
but it's silently ignored because `RobotClientConfig` in lerobot 0.4.0 doesn't
have that field — only a warning at launch time:

```
[WARN] obs_similarity_atol is not yet supported in this LeRobot version.
```

**Fix:** Monkey-patched the function in the installed lerobot:

```python
# /home/ubuntu/.local/lib/python3.10/site-packages/lerobot/async_inference/helpers.py:279
def observations_similar(obs1, obs2, lerobot_features, atol=1) -> bool:
    """PATCHED: filter disabled for rosetta async inference."""
    del obs1, obs2, lerobot_features, atol
    return False
```

The patch is contained — `observations_similar` is only called by the async
inference path; `lerobot-record`, training, and dataset code don't use it.

### 2. Action-chunk aggregator was blending incompatible chunks

`rosetta_client.yaml` had `aggregate_fn_name: weighted_average`, which on
overlapping timesteps applies:

```python
queue[t] = 0.3 * old_chunk[t] + 0.7 * new_chunk[t]
```

But the trained model has `temporal_ensemble_coeff: null` — it was **not**
trained for its chunks to be blended. Blending two trajectories that were
predicted from slightly different observations produces actions neither
plan intended → muted, oscillatory motion.

**Fix:** `aggregate_fn_name: latest_only` so new chunks cleanly replace
overlapping queue entries.

### 3. Chunk size / threshold was preventing trajectory progress (biggest fix)

Old settings:
```yaml
actions_per_chunk: 30
chunk_size_threshold: 0.95
```

This meant:
- Server returned 30 actions per chunk
- Client requested a new chunk as soon as queue dropped below 28 actions
- Robot only ever executed actions #0–1 of any chunk before it was replaced
- Action #0 of an ACT chunk is "from current state, the smallest possible step
  toward the goal" — so the robot perpetually tried to "start" the trajectory
  from its current position and never progressed

`lerobot-record`'s `select_action` consumes all `n_action_steps=100` actions
of each chunk before re-predicting. Rosetta was effectively restarting from
step 0 every ~67 ms.

**Fix:**
```yaml
actions_per_chunk: 100      # full chunk, like lerobot-record
chunk_size_threshold: 0.0   # only request new chunk when queue is empty
```

This makes Rosetta consume the entire 3.3 s trajectory before re-predicting,
matching `lerobot-record`'s in-process behavior.

### 4. Orphan policy_server process bound to port 8080

`netstat`/`/proc/net/tcp6` revealed two `policy_server` processes both bound
to 127.0.0.1:8080 via `SO_REUSEADDR`:

- PID 8315 launched at 11:25 (before the helpers.py patch — filter still active)
- PID 31243 launched at 11:47 by current rosetta_client (filter patched)

Incoming gRPC connections could land on either listener depending on kernel
load balancing. Tracking it down via per-PID socket inode mapping:

```bash
awk '$4=="0A" && $2 ~ /:1F90$/ {print $10}' /proc/net/tcp6  # listening inodes
ls -l /proc/<pid>/fd | grep socket:\\[<inode>\\]            # who owns each
```

**Fix:** `kill 8315` and verified only one listener remained.

### 5. Calibration mismatch between ROS stack and lerobot.SO101Follower (current root cause)

After the first four fixes the arm executed a full coherent trajectory but
poorly — the wrist stayed tilted during the grab, and the policy seemed to
skip the reach-into-shelf phase from certain starting poses.

Held the arm in one pose and read it both ways. ROS values (radians) converted
to degrees:

| Joint | ROS reading (°) | lerobot reading (°) | Δ = lerobot − ROS |
|---|---:|---:|---:|
| shoulder_pan | −5.27 | +14.20 | **+19.47°** |
| shoulder_lift | −87.81 | −87.01 | +0.80° |
| elbow_flex | +90.70 | +99.64 | **+8.94°** |
| wrist_flex | −7.56 | −3.25 | **+4.32°** |
| wrist_roll | **−91.67** | **−55.30** | **+36.37°** ⚠ |
| gripper | +1.14 | +17.34 | **+16.19°** ⚠ |

Both stacks load the same calibration JSON (`gix-follower4.json`) but
`feetech_ros2_driver` + xacro calibration in `soa_bringup` apply it
differently than `lerobot.SO101Follower`. The 36° error on `wrist_roll`
directly explains the persistent wrist tilt symptom; the 16° error on the
gripper explains "policy reads gripper as closed at start" when it's
actually open. ACT is an absolute-position controller, so the bad joint
frame compounds in both directions: wrong observation in, wrong physical
position out.

**Workaround:** A contract-level decoder/encoder shim that adds the per-joint
offset on the observation side and subtracts it on the action side. Lives in
[`ros2_ws/src/soa_ros2/soa_bringup/soa_bringup/policy_calibration.py`](ros2_ws/src/soa_ros2/soa_bringup/soa_bringup/policy_calibration.py):

```python
JOINT_OFFSETS_DEG: dict[str, float] = {
    "shoulder_pan":  19.47,
    "shoulder_lift":  0.80,
    "elbow_flex":     8.94,
    "wrist_flex":     2.16,
    "wrist_roll":    36.37,
    "gripper":       16.19,
}

def decode_state(msg, spec) -> np.ndarray:
    """JointState (ROS radians) → policy-frame degrees, with offset added."""
    ...

def encode_action(action_vec, spec, stamp_ns=None):
    """Policy-frame degrees → Float64MultiArray of ROS-frame radians."""
    ...
```

Wired into the contract:

```yaml
observations:
  - key: observation.state
    ...
    decoder: soa_bringup.policy_calibration:decode_state
    # NOTE: no unit_conversion: rad2deg — decoder does both at once

actions:
  - key: action
    publish: { topic: /follower/arm_fwd_controller/commands, ... }
    selector: { names: [...arm joints...] }
    encoder: soa_bringup.policy_calibration:encode_action
  - key: action
    publish: { topic: /follower/gripper_fwd_controller/commands, ... }
    selector: { names: [position.gripper] }
    encoder: soa_bringup.policy_calibration:encode_action
```

This works for joints where the discrepancy is a pure constant offset. If
multi-pose measurement shows the delta varies with pose on some joint, that
joint has a scale (or sign) mismatch too and the shim's add-a-constant model
is insufficient — the proper fix is in `feetech_ros2_driver` /
`calibration_loader.py`.

---

## Why the Policy Worked Through `lerobot-record` But Not Rosetta

`lerobot-record` reads and writes through the same `lerobot.SO101Follower`
that captured the training data, so the joint frame at inference time
matches the joint frame at training time. Rosetta interposes the ROS2 stack
(`ros2_control_node` + `feetech_ros2_driver` + xacro calibration loader)
between the policy and the servos. That stack produces a different joint
frame, and the policy has no way to know it's been shifted.

The four async-pipeline issues (1–4) compounded the symptom — the model was
running open-loop in 67 ms windows and seeing only every ~30th observation,
all of which made the behavior look like "model is broken" rather than
"calibration is off." Once those were fixed, the underlying calibration
mismatch (issue 5) became cleanly visible.

---

## Files Touched

| File | Change |
|---|---|
| `ros2_ws/src/rosetta/params/rosetta_client.yaml` | `actions_per_chunk: 100`, `chunk_size_threshold: 0.0`, `aggregate_fn_name: latest_only` |
| `/home/ubuntu/.local/lib/python3.10/site-packages/lerobot/async_inference/helpers.py` | `observations_similar` patched to `return False` |
| `ros2_ws/src/rosetta/contracts/so_101_grab_and_pour.yaml` | Custom `decoder` / `encoder` references, removed `unit_conversion: rad2deg` |
| `ros2_ws/src/soa_ros2/soa_bringup/soa_bringup/policy_calibration.py` | **NEW** — calibration shim module |
| `calibrate_interactive.py` | **NEW** — single-session interactive offset measurement script |

---

## Useful Diagnostic Commands

```bash
# Check publish rates
ros2 topic hz /follower/joint_states
ros2 topic hz /follower/image_raw/compressed
ros2 topic hz /static_camera/overhead_cam/color/image_raw/compressed
ros2 topic hz /follower/arm_fwd_controller/commands

# Inspect joint values (radians; multiply by 57.2958 for degrees)
ros2 topic echo /follower/joint_states --once

# Inspect commanded values
ros2 topic echo /follower/arm_fwd_controller/commands
ros2 topic echo /follower/gripper_fwd_controller/commands

# Read via lerobot (with soa_bringup STOPPED so /dev/ttyACM3 is free)
python3 -c "
from pathlib import Path
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
cfg = SO101FollowerConfig(port='/dev/ttyACM3', id='gix-follower4',
    calibration_dir=Path('/home/ubuntu/techin517/huggingface/lerobot/calibration/robots/so101_follower'))
r = SO101Follower(cfg); r.connect(calibrate=False); print(r.get_observation()); r.disconnect()
"

# Find who owns port 8080 (gRPC listens via IPv6, not v4)
awk '$4=="0A" && $2 ~ /:1F90$/ {print $10}' /proc/net/tcp6  # listening inodes
ls -l /proc/<pid>/fd | grep "socket:\[<inode>\]"             # per-pid owner

# Server-side filter activity (look for "too similar" lines)
grep -c "Skipping observation" /home/ubuntu/techin517/ros2_ws/logs/policy_server_*.log
grep -c "Running inference"     /home/ubuntu/techin517/ros2_ws/logs/policy_server_*.log
```

---

## Long-term TODO

1. Fix the calibration application in `feetech_ros2_driver` and/or
   `soa_bringup/calibration_loader.py` so the ROS stack produces the same
   joint frame as `lerobot.SO101Follower`. Remove the
   `policy_calibration.py` shim afterward.

2. Upgrade LeRobot to a version that exposes `obs_similarity_atol` on
   `RobotClientConfig` (currently blocked by Python 3.10 vs 3.12). Then
   revert the `helpers.py` patch.

3. If the calibration mismatch turns out to be non-linear (per-joint Δ
   varies with pose), extend `policy_calibration.py` to apply a linear or
   piecewise correction instead of a constant offset. `calibrate_interactive.py
   --linear-fit` reports the slope.

4. Reconsider the camera path: training used direct OpenCV captures at
   640×480, runtime uses the RealSense overhead at 1280×720 squashed to
   480×640. Either drop the RealSense to 640×480 (conflicts with YOLO's
   bottle detection at higher res) or add a republisher node that exposes
   a lower-res topic specifically for the policy.

---

## Final Runbook

Pre-position the arm near a bottle (gripper open, wrist pointed down — same
as you would for `lerobot-record`).

```bash
# Terminal 1
ros2 launch soa_bringup soa_bringup.launch.py

# Terminal 2
ros2 launch rosetta rosetta_client_launch.py \
  contract_path:=/home/ubuntu/techin517/ros2_ws/src/rosetta/contracts/so_101_grab_and_pour.yaml \
  pretrained_name_or_path:=/home/ubuntu/techin517/outputs/train/grab_and_pour_right_v5/checkpoints/last/pretrained_model \
  policy_type:=act

# Terminal 3
ros2 action send_goal /run_policy rosetta_interfaces/action/RunPolicy "{prompt: 'grab and pour'}"
```

To re-measure the calibration offsets (e.g. after re-calibrating servos or
moving to a new robot):

```bash
python3 /home/ubuntu/techin517/calibrate_interactive.py
# Move arm through 3–5 poses, ENTER at each, q+ENTER to finish.
# Stop soa_bringup, reproduce same poses, ENTER at each.
# Paste the printed JOINT_OFFSETS_DEG block into policy_calibration.py.
```
