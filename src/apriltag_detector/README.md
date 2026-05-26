# apriltag_detector

ROS 2 integration package for AprilTag-based pallet detection.

## Runtime pieces

- `apriltag_detector/direct_detector.py` is a lifecycle node. When active, it consumes `/camera/image` and `/camera/camera_info` directly, runs the AprilTag detector, publishes `/apriltag_detector/detections`, and broadcasts `base_link -> pallet_*_tag`.
- `apriltag_detector/detection_monitor.py` logs whether detections are absent, empty, or populated.
- `config/direct_detector.yaml` configures pallet tag IDs, tag sizes, and TF frame names.
- `launch/apriltag_detector.launch.py` starts the direct detector and detection monitor.

The direct detector uses the same AprilRobotics AprilTag library, but avoids `image_transport` synchronization issues by processing the latest camera image and camera_info pair itself.

By default the launch configures the detector into `inactive`; it does not publish detections or TF until activated.

Activate detections:

```bash
ros2 lifecycle set /apriltag_direct_detector activate
```

Deactivate detections:

```bash
ros2 lifecycle set /apriltag_direct_detector deactivate
```

Start already active:

```bash
ros2 launch apriltag_detector apriltag_detector.launch.py activate_detector:=true
```

Check that the camera and detections are running:

```bash
ros2 topic hz /camera/image
ros2 topic echo /apriltag_detector/detections --once
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo base_link pallet_b_south_00_tag
```
