# robot_bringup
Launch and config files for the robot

## ROS 2 straight-line global planner

`straight_line_global_planner.launch.py` replaces GBPlanner for stacks where
the downstream CBF is responsible for collision avoidance. It accepts a ROS 2
`geometry_msgs/PoseStamped` goal on `/goal_pose` and publishes a two-pose ROS 2
`nav_msgs/Path` from the latest `/rmf/odom` position directly to the goal on
`/gbplanner_path`. It republishes from the latest position every two seconds by
default; set `replan_interval` to change that period. The output topic and
message type intentionally match the path consumed by NMPC.

The custom Docker Compose stack exposes the same setting through
`GLOBAL_PLANNER_REPLAN_INTERVAL`.

## ROS 1 automatic waypoint follower

Edit `config/ros1/waypoint_follower/waypoints.yaml`. Coordinates are in the
GBPlanner `world` frame. A waypoint may specify `yaw` in radians, `yaw_deg` in
degrees, or quaternion fields `qx`, `qy`, `qz`, and `qw`.

With GBPlanner and PCI already running, start the mission with:

```bash
roslaunch robot_bringup waypoint_follower.launch autostart:=true
```

Use another YAML file with:

```bash
roslaunch robot_bringup waypoint_follower.launch \
  waypoint_file:=/absolute/path/to/waypoints.yaml autostart:=true
```

For an operator-controlled start, launch without autostart and trigger it when
the robot is ready:

```bash
roslaunch robot_bringup waypoint_follower.launch
rosservice call /waypoint_follower/start "{}"
```

The follower switches GBPlanner to waypoint mode and publishes each pose to
`/move_base_simple/goal`. This only loads the first target: it does not start
PCI or command the vehicle directly. Use the **Start Planner** button after
launching the follower to begin target-reach planning and motion.

The next target is published when `/rmf/odom` is within `reached_distance`
(2 m by default) of the current target. This matches GBPlanner's configured
`local_navigation_reaching_radius`. After the operator's first planner start
has produced a real `/gbplanner_path`, the follower may re-trigger PCI between
targets so that one explicit planner start runs the complete sequence. It will
never trigger the initial planner start itself.
