### Launch soa_bringup
ros2 launch soa_bringup soa_bringup.launch.py leader:=true display:=true

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