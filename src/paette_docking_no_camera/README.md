# paette_docking_no_camera

`paette_docking_no_camera` is a Nav2-based pallet docking module. It reads the
AprilTag TF that is already published by the AprilTag detector stack, builds a
small docking plan, publishes visualization into RViz, turns with a bounded yaw
loop, and executes straight segments with Nav2 `DriveOnHeading`.

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

The node does not call Nav2 `FollowPath`. Docking is intentionally executed as a
small sequence of yaw turns and primitive Nav2 drive actions:

```text
WAIT_FOR_TAG
  -> optional RETREAT
  -> PLANNING
  -> APPROACH_PREFINAL: yaw turn + DriveOnHeading
  -> ALIGN_TO_PALLET: yaw turn
  -> FINAL_DOCK: DriveOnHeading
  -> DONE
```

Detailed flow:

1. The service receives `start` and starts a worker thread. The service returns
   immediately with the current status while the motion continues in the node.
2. The node looks up `target_frame -> tag_frame` TF. With the default config it
   checks all `tag_frame_candidates` and selects the visible tag with the
   smallest initial angle in the docking frame.
3. In `reverse` mode the tag vector is interpreted like the old
   `palette_docking` controller: the pallet is approached along negative
   `base_link.x`. This matches the forklift geometry where the forks are behind
   `base_link`.
4. If the pallet distance is smaller than `close_distance_threshold_m`, the
   robot first drives away from the pallet by `retreat_distance_m` using
   `DriveOnHeading`.
5. The node computes a prefinal point in `global_frame`. The point lies on the
   current robot-to-pallet line and is
   `prefinal_distance_from_pallet_m` before the pallet tag.
6. The node publishes the computed prefinal line to `path_topic` and RViz
   markers to `marker_topic`. This path is visualization only; it is not sent to
   Nav2 `FollowPath`.
7. For the prefinal segment the node turns with a bounded yaw loop published to
   `cmd_vel_topic`. The yaw loop may include a small rolling assist so the
   simulated steered wheel can change angle while moving. In `reverse` mode the
   requested yaw is flipped by pi, so `base_link.x` points away from the pallet
   and the rear forks face the pallet.
8. After the yaw turn, the node refreshes the pallet TF and recomputes the
   prefinal segment, then calls `DriveOnHeading` for
   `plan.segment_distance`. In `reverse` mode this uses negative `target.x` and
   negative speed, so the robot drives backwards to the prefinal point.
9. At the prefinal point the node refreshes the tag TF if possible, computes the
   final docking yaw from the current robot pose to the pallet tag, and runs the
   same bounded yaw loop again.
10. Before final entry, the node refreshes the tag distance once more. If the
    yaw rolling assist moved the robot farther away, the final drive distance is
    increased to at least the latest tag distance.
11. The node calls `DriveOnHeading` to drive straight under the pallet. In
    default `reverse` mode this final motion is also backwards.

The practical consequence is that docking no longer depends on the Nav2 path
controller, regulated pure pursuit, goal checkers, or Nav2 `Spin` result
completion. Straight-line distances still use Nav2 `DriveOnHeading`; yaw turns
go through the normal `/cmd_vel_nav_raw -> collision_monitor -> cmd_vel_arcestrator`
velocity chain.

## Main Parameters

- `dock_motion_mode`: `reverse` by default. Use this for the forklift because
  the forks are behind `base_link`.
- `tag_frame`: explicit pallet tag frame. Empty means auto-select from
  `tag_frame_candidates`.
- `prefinal_distance_from_pallet_m`: distance from pallet tag to the prefinal
  stop point.
- `final_drive_distance_m`: straight final entry distance under the pallet.
- `close_distance_threshold_m`: if the pallet is closer than this, retreat
  first.
- `retreat_distance_m`: retreat distance before replanning when too close.
- `max_initial_angle_rad`: rejects docking if the initially selected tag needs
  too much heading correction.
- `yaw_tolerance_rad`: yaw turn completion/skip threshold.
- `yaw_timeout_acceptance_rad`: if a yaw turn times out but the remaining yaw is
  below this value, the node logs a warning and continues instead of failing.
- `cmd_vel_topic`: topic for docking yaw turns, default `/cmd_vel_nav_raw`.
- `yaw_angular_speed_radps`: bounded angular speed for yaw turns.
- `yaw_linear_assist_mps`: small away-from-pallet linear velocity during yaw
  turns. It helps the Gazebo steering joint move because the wheel is allowed to
  roll while changing steering angle. Keep this below the forward stop polygon
  threshold in `collision_monitor_params.yaml`; the default is `0.03`.
- `yaw_control_frequency_hz`: control loop rate for yaw turns.
- `retreat_speed_mps`, `prefinal_drive_speed_mps`,
  `final_drive_speed_mps`: speeds for the three `DriveOnHeading` stages.
- `spin_timeout_sec`: yaw turn timeout. `drive_timeout_sec`: `DriveOnHeading`
  action result timeout.

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
