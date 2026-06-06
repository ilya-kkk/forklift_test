# paette_docking_no_camera

`paette_docking_no_camera` is a Nav2-based pallet docking module. It reads the
AprilTag TF that is already published by the AprilTag detector stack, builds a
small docking plan, publishes visualization into RViz, and executes the plan
through Nav2 actions instead of publishing raw velocity commands.

The package name follows the current folder name. The ROS node and service use
the clearer `palette_docking_no_camera` naming:

```bash
ros2 run paette_docking_no_camera palette_docking_no_camera \
  --ros-args --params-file src/paette_docking_no_camera/config/palette_docking_no_camera.yaml
```

## Service API

The node exposes `forklift_interfaces/srv/StringWithJson` on
`/palette_docking_no_camera/control` by default.

Start docking:

```bash
ros2 service call /palette_docking_no_camera/control forklift_interfaces/srv/StringWithJson \
  "{message: '{\"command\":\"start\"}'}"
```

Start with one explicit tag and per-run distance overrides:

```bash
ros2 service call /palette_docking_no_camera/control forklift_interfaces/srv/StringWithJson \
  "{message: '{\"command\":\"start\",\"tag_frame\":\"pallet_b_south_00_tag\",\"close_distance_threshold_m\":0.8,\"prefinal_distance_from_pallet_m\":1.0,\"final_drive_distance_m\":1.0}'}"
```

Status:

```bash
ros2 service call /palette_docking_no_camera/control forklift_interfaces/srv/StringWithJson \
  "{message: '{\"command\":\"status\"}'}"
```

Cancel:

```bash
ros2 service call /palette_docking_no_camera/control forklift_interfaces/srv/StringWithJson \
  "{message: '{\"command\":\"cancel\"}'}"
```

Supported commands are `start`, `dock`, `cancel`, `stop`, `status`, and `reset`.
The request may also contain `{"enabled": true}` or `{"enabled": false}`.

## Runtime Behavior

1. The service receives `start` and starts a worker thread. The service returns
   immediately with the current status while the motion continues in the node.
2. The node looks up `target_frame -> tag_frame` TF. With the default config it
   checks all `tag_frame_candidates` and selects the visible tag with the
   smallest initial angle in the docking frame.
3. In `reverse` mode the tag vector is interpreted like the old
   `palette_docking` controller: the pallet is approached along negative
   `base_link.x`.
4. If the pallet distance is smaller than `close_distance_threshold_m`, the
   robot first drives away from the pallet by `retreat_distance_m` using Nav2
   `DriveOnHeading`.
5. The node computes a prefinal point in `global_frame`. This point lies on the
   current robot-to-pallet line and is
   `prefinal_distance_from_pallet_m` before the pallet tag.
6. The node publishes the prefinal path to `path_topic` and RViz markers to
   `marker_topic`. Markers include the pallet tag point, current robot point,
   prefinal point, path line, and expected prefinal heading.
7. The robot spins toward the prefinal segment with Nav2 `Spin`.
8. The robot drives to the prefinal point. Short segments use
   `DriveOnHeading`; longer segments use Nav2 `FollowPath`.
9. At the prefinal point the node optionally refreshes the tag TF, computes the
   final docking yaw, and spins to face the pallet in the configured docking
   direction.
10. The robot drives straight under the pallet by `final_drive_distance_m` using
    Nav2 `DriveOnHeading`. In default `reverse` mode this command uses negative
    `target.x` and negative speed.

## RViz Topics

- `/palette_docking_no_camera/path` publishes the prefinal `nav_msgs/Path`.
- `/palette_docking_no_camera/markers` publishes `visualization_msgs/MarkerArray`.

Both publishers use transient local QoS so RViz can show the latest plan after
the display is added.

## Notes

This implementation intentionally keeps the docking geometry simple. It uses the
observed robot-to-tag line as the pallet approach axis. If the pallet model later
needs a fixed approach direction independent of the current robot pose, the next
step should be to derive that axis from the AprilTag orientation or from a
pallet-specific static transform.
