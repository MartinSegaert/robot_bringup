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

## ROS 2 automatic waypoint follower

Edit `config/ros2/waypoint_follower/waypoints.yaml`. Coordinates are in the
ROS 2 `map` frame. A waypoint may specify `yaw` in radians, `yaw_deg` in
degrees, or quaternion fields `qx`, `qy`, `qz`, and `qw`.

With the ROS 2 straight-line planner running, start the mission with:

```bash
ros2 launch robot_bringup waypoint_follower.launch.py autostart:=true
```

Use another YAML file with:

```bash
ros2 launch robot_bringup waypoint_follower.launch.py \
  waypoint_file:=/absolute/path/to/waypoints.yaml autostart:=true
```

For an operator-controlled start, launch without autostart and trigger it when
the robot is ready:

```bash
ros2 launch robot_bringup waypoint_follower.launch.py
ros2 service call /waypoint_follower/start std_srvs/srv/Trigger "{}"
```

The follower publishes each target on `/goal_pose`. The straight-line planner
then updates `/gbplanner_path` for NMPC. The next target is published when
`/rmf/odom` is within `reached_distance` (2 m by default) of the active target.
