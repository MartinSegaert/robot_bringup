# robot_bringup
Launch and config files for the robot

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

The follower sends each pose through `/pci_to_waypoint`. A `true` message on
`/gbplanner_status` advances the mission; a terminal `false` retries according
to `max_retries` and then stops. Do not run another PCI planning behavior at the
same time because `gbplanner_status` is a shared planner-status topic.
