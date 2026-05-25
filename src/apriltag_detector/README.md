# apriltag_detector

ROS 2 integration package for AprilTag-based pallet detection.

## Runtime pieces

- `apriltag_detector/image_gate.py` gates `/camera/image` and `/camera/camera_info` behind `/apriltag_detector/control`.
- When enabled, the gate republishes the latest received camera pair to `/apriltag_detector/image` and `/apriltag_detector/camera_info` at `publish_rate_hz`.
- `apriltag_detector/direct_detector.py` consumes the latest gated camera pair, runs the AprilTag detector, publishes `/apriltag_detector/detections`, and broadcasts `base_link -> pallet_*_tag`.
- `apriltag_detector/detection_monitor.py` logs whether detections are absent, empty, or populated.
- `config/direct_detector.yaml` configures pallet tag IDs, tag sizes, and TF frame names.
- `launch/apriltag_detector.launch.py` starts the gate, direct detector, and detection monitor.

The direct detector uses the same AprilRobotics AprilTag library, but avoids `image_transport` synchronization issues by processing the latest gated image and camera_info pair itself.

Enable detections:

```bash
ros2 service call /apriltag_detector/control forklift_interfaces/srv/StringWithJson "{message: '{\"enabled\": true}'}"
```

Check that the gated stream is running:

```bash
ros2 topic hz /apriltag_detector/image
ros2 topic echo /apriltag_detector/detections --once
ros2 topic echo /tf --once
ros2 run tf2_ros tf2_echo base_link pallet_b_south_00_tag
```

Disable detections:

```bash
ros2 service call /apriltag_detector/control forklift_interfaces/srv/StringWithJson "{message: '{\"enabled\": false}'}"
```
