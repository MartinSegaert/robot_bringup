# robot_bringup
Launch and config files for the robot

## ROS 2 straight-line global planner

`straight_line_global_planner.launch.py` replaces GBPlanner for stacks where
the downstream CBF is responsible for collision avoidance. It accepts a ROS 2
`geometry_msgs/PoseStamped` goal on `/goal_pose` and publishes an equally
spaced ROS 2 `nav_msgs/Path` from the latest `/rmf/odom` position directly to
the goal on `/gbplanner_path`. Every pose has its yaw facing the goal. The
output topic and message type intentionally match the path consumed by NMPC.

The planner immediately publishes for each new goal. It then replans only when
odometry is more than `retrigger_distance` away from the previously published
path. All tunable parameters, including `segment_length` and
`retrigger_distance`, are in
`config/ros2/custom/custom_global_planner.yaml`. The launch file and custom
Docker Compose stack load this file by default.

The planner also publishes a live allowed reference speed on
`/sdf_nmpc/reference_speed`. It uses the smaller of the distance to the closest
finite lidar point and the remaining 3D distance to the goal. The speed is
`min_speed` at or below `min_distance`, `max_speed` at or above `max_distance`,
and linearly interpolated between them. The NMPC reference generator consumes
this topic and applies a new speed on its next reference horizon; `ref.vref` in
the NMPC YAML remains its startup value.

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
At startup, the follower waits for the planner's goal subscription to be
discovered before publishing the first waypoint.
