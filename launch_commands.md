### Launch soa_bringup
ros2 launch soa_bringup soa_bringup.launch.py leader:=true display:=true

### Launch `yolo_ros` with only detection
```
ros2 launch yolo_bringup yolo.launch.py \
  model:=/home/ubuntu/techin517/alcohol_detection_model/run/weights/best.pt \
  input_image_topic:=/static_camera/overhead_cam/color/image_raw
``

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
