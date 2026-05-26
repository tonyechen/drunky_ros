### Launch soa_bringup
ros2 launch soa_bringup soa_bringup.launch.py leader:=true display:=true

### Launch `yolo_ros` with only detection
```
ros2 launch yolo_bringup yolo.launch.py \
  model:=/home/ubuntu/techin517/alcohol_detection_model/run/weights/best.pt \
  input_image_topic:=/static_camera/overhead_cam/color/image_raw
```

### Launch `yolo_ros` with depth detection
```
ros2 launch yolo_bringup yolo.launch.py \
  model:=/home/ubuntu/techin517/alcohol_detection_model/run/weights/best.pt \
  input_image_topic:=/static_camera/overhead_cam/color/image_raw \
  input_depth_topic:=/static_camera/overhead_cam/aligned_depth_to_color/image_raw \
  input_depth_info_topic:=/static_camera/overhead_cam/aligned_depth_to_color/camera_info \
  use_3d:=True \
  use_debug:=True \
  target_frame:=overhead_camoverhead_cam_color_optical_frame
```
# Running ACT Policy
```
lerobot-record   --robot.type=so101_follower   --robot.port=/dev/ttyACM3   --robot.id=gix-follower4   --robot.cameras="{ arm: {type: opencv, index_or_path: '/dev/video0', width: 640, height: 480, fps: 30}, middle: {type: opencv, index_or_path: '/dev/video8', width: 640, height: 480, fps: 30}}"   --display_data=false   --dataset.repo_id=${HF_USER}/eval_grab_and_pour_right_v5   --dataset.single_task="grab and pour"   --policy.path=/home/ubuntu/techin517/outputs/train/grab_and_pour_right_v5/checkpoints/last/pretrained_model   --dataset.episode_time_s=30

```

# Running grab_and_pour_v5 ACT with Rosetta
### Launch soa_bringup
```
ros2 launch soa_bringup soa_bringup.launch.py
```

### Launch Rosetta
```
ros2 launch rosetta rosetta_client_launch.py   contract_path:=/home/ubuntu/techin517/ros2_ws/src/rosetta/contracts/so_101_grab_and_pour.yaml   pretrained_name_or_path:=/home/ubuntu/techin517/outputs/train/grab_and_pour_right_v5/checkpoints/last/pretrained_model   policy_type:=act
```

### Send action
```
ros2 action send_goal /run_policy   rosetta_interfaces/action/RunPolicy "{prompt: 'grab and pour'}"
```

# Pose Estimation
Right Arm hand-eye calibration.

Values are the composed `base_link → camera URDF root` transform (hand-eye
calibration result + the Realsense's internal `color_optical → cam_link`
offset). Publishing to the camera URDF root (which has no other parent)
avoids the TF conflict with the Realsense's internal chain.

### With soa_moveit_bringup (no namespace prefix → parent is `base_link`):
```
ros2 run tf2_ros static_transform_publisher \
  --x 0.044184 --y 0.168245 --z 0.332111 \
  --qx -0.004642 --qy 0.363628 --qz 0.009791 --qw 0.931481 \
  --frame-id base_link \
  --child-frame-id overhead_camoverhead_cam_overhead_cam_link
```

### With soa_bringup (frames are namespaced under `follower/`):
```
ros2 run tf2_ros static_transform_publisher \
  --x 0.044184 --y 0.168245 --z 0.332111 \
  --qx -0.004642 --qy 0.363628 --qz 0.009791 --qw 0.931481 \
  --frame-id follower/base_link \
  --child-frame-id overhead_camoverhead_cam_overhead_cam_link
```

# Running Pose Estimation Action Server

### 1) MoveIt + ros2_control + RSP + static base_link TF
```
ros2 launch bi_soa_moveit_config bi_soa_moveit_bringup.launch.py
```

### 2) Camera → base_link static TF (soa_moveit_bringup version, no `follower/` prefix)
```
ros2 run tf2_ros static_transform_publisher \
  --x 0.044184 --y 0.168245 --z 0.332111 \
  --qx -0.004642 --qy 0.363628 --qz 0.009791 --qw 0.931481 \
  --frame-id base_link \
  --child-frame-id overhead_camoverhead_cam_overhead_cam_link
```

### 3) YOLO 2D detection (no `use_3d` — `bottle_depth_node` does the 3D step)
```
ros2 launch yolo_bringup yolo.launch.py \
  model:=/home/ubuntu/techin517/alcohol_detection_model/run/weights/best.pt \
  input_image_topic:=/static_camera/overhead_cam/color/image_raw
```

### 4) `bottle_depth_node` — replaces yolo_ros's detect_3d_node
Samples the nearest 10% of depth pixels in each YOLO bbox, so the table no
longer pulls the reported Z back and the lateral position stops getting
magnified. Publishes on `/yolo/detections_3d` so downstream nodes need no
changes.
```
ros2 run bottle_depth bottle_depth_node --ros-args \
  -p target_frame:=base_link \
  -r detections:=/yolo/detections \
  -r depth_image:=/static_camera/overhead_cam/aligned_depth_to_color/image_raw \
  -r depth_info:=/static_camera/overhead_cam/aligned_depth_to_color/camera_info \
  -r detections_3d:=/yolo/detections_3d
```

Tunable params: `nearest_fraction` (default 0.10), `min_samples` (default 5),
`depth_image_units_divisor` (default 1000). With `soa_bringup`, set
`target_frame:=follower/base_link`.

### 5) MoveToPose action server
```
ros2 run soa_functions move_to_pose_server
```

### 6) Pose estimation action server
```
ros2 run soa_functions pose_estimation_server
```

### 7) Trigger pose estimation for a shooter bottle
Supported: `coke`, `gin`, `ginger_beer`, `orange_juice`, `tequila`, `tonic_water`, `vodka`, `whiskey`.
```
ros2 action send_goal /estimate_shooter_pose \
  soa_interfaces/action/EstimateShooterPose \
  "{shooter_name: 'vodka'}" --feedback
```

# Go To Drink

Runs the full pipeline: calls `estimate_shooter_pose`, transforms the detection
into `base_link`, builds an end-effector goal (wrist parallel to ground, with
configurable x/y/z standoff), and sends it to `move_to_pose`.

Prerequisites — steps 1–6 from "Running Pose Estimation Action Server" above
(MoveIt bringup, camera TF, YOLO 2D, `bottle_depth_node`, `move_to_pose_server`,
`pose_estimation_server`).

### Run
```
ros2 run soa_apps go_to_drink --ros-args -p shooter_name:=gin
```

### Optional: override standoff offsets (meters)
`offset` is the radial standoff pulled along the XY direction from the bottle
back toward the base_link origin. `z_offset` is subtracted from the bottle's Z.
```
ros2 run soa_apps go_to_drink --ros-args \
  -p shooter_name:=gin \
  -p offset:=0.13 \
  -p z_offset:=-0.07
```

### With soa_bringup (namespaced frames) override base_frame
```
ros2 run soa_apps go_to_drink --ros-args \
  -p shooter_name:=coke \
  -p base_frame:=follower/base_link
```
