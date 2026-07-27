#!/usr/bin/env python3

"""Send a configured sequence of target-reach goals to GBPlanner."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger, TriggerResponse


class WaypointFollower:
    def __init__(self):
        self.waypoints = self._load_waypoints()
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.inter_waypoint_delay = max(
            0.0, float(rospy.get_param("~inter_waypoint_delay", 0.5))
        )
        self.reached_distance = float(
            rospy.get_param("~reached_distance", 2.0)
        )
        if self.reached_distance <= 0.0:
            raise ValueError("~reached_distance must be positive")
        self.goal_topic = rospy.get_param(
            "~goal_topic", "/move_base_simple/goal"
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/rmf/odom"
        )
        self.operation_mode_service_name = rospy.get_param(
            "~operation_mode_service", "/gbplanner/switch_operation_mode"
        )
        self.planner_path_topic = rospy.get_param(
            "~planner_path_topic", "/gbplanner_path"
        )
        self.planner_start_service_name = rospy.get_param(
            "~planner_start_service",
            "/planner_control_interface/std_srvs/automatic_planning",
        )

        self.index = 0
        self.running = False
        self.waiting_for_reach = False
        self.planner_started = False
        self.send_pending = False
        self.send_timer = None
        self.restart_timer = None
        self.position = None

        self.goal_publisher = rospy.Publisher(
            self.goal_topic, PoseStamped, queue_size=1, latch=True
        )
        self.odometry_subscriber = rospy.Subscriber(
            self.odometry_topic, Odometry, self._odometry_callback, queue_size=1
        )
        self.path_subscriber = rospy.Subscriber(
            self.planner_path_topic, Path, self._path_callback, queue_size=1
        )
        self.start_server = rospy.Service("~start", Trigger, self._start_callback)

        rospy.loginfo(
            "Waypoint follower: waiting for GBPlanner mode service %s",
            self.operation_mode_service_name,
        )
        rospy.wait_for_service(self.operation_mode_service_name)
        self.set_operation_mode = rospy.ServiceProxy(
            self.operation_mode_service_name, SetBool, persistent=False
        )
        self.start_planner = rospy.ServiceProxy(
            self.planner_start_service_name, Trigger, persistent=False
        )

        rospy.loginfo(
            "Waypoint follower ready with %d waypoint(s); publishing GBPlanner "
            "goals on %s; start service is %s/start",
            len(self.waypoints),
            self.goal_topic,
            rospy.get_name(),
        )
        if rospy.get_param("~autostart", False):
            if self._start_mission():
                self._schedule_send(0.5)

    @staticmethod
    def _number(value, field, waypoint_index):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(
                "waypoint {} field '{}' must be a number".format(
                    waypoint_index + 1, field
                )
            )
        return float(value)

    def _load_waypoints(self):
        raw_waypoints = rospy.get_param("~waypoints", None)
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("~waypoints must be a non-empty YAML list")

        waypoints = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError(
                    "waypoint {} must be a YAML mapping".format(index + 1)
                )
            if "x" not in waypoint or "y" not in waypoint:
                raise ValueError(
                    "waypoint {} must define x and y".format(index + 1)
                )

            parsed = {
                "name": str(waypoint.get("name", "waypoint_{}".format(index + 1))),
                "x": self._number(waypoint["x"], "x", index),
                "y": self._number(waypoint["y"], "y", index),
                "z": self._number(waypoint.get("z", 0.0), "z", index),
            }

            quaternion_fields = ("qx", "qy", "qz", "qw")
            has_quaternion = any(field in waypoint for field in quaternion_fields)
            has_yaw = "yaw" in waypoint or "yaw_deg" in waypoint
            if has_quaternion and has_yaw:
                raise ValueError(
                    "waypoint {} cannot define both yaw and a quaternion".format(
                        index + 1
                    )
                )

            if has_quaternion:
                parsed.update(
                    {
                        field: self._number(
                            waypoint.get(field, 1.0 if field == "qw" else 0.0),
                            field,
                            index,
                        )
                        for field in quaternion_fields
                    }
                )
                norm = math.sqrt(
                    sum(parsed[field] ** 2 for field in quaternion_fields)
                )
                if norm < 1e-9:
                    raise ValueError(
                        "waypoint {} quaternion cannot be zero".format(index + 1)
                    )
                for field in quaternion_fields:
                    parsed[field] /= norm
            else:
                if "yaw_deg" in waypoint:
                    yaw = math.radians(
                        self._number(waypoint["yaw_deg"], "yaw_deg", index)
                    )
                else:
                    yaw = self._number(waypoint.get("yaw", 0.0), "yaw", index)
                parsed.update(
                    {
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": math.sin(yaw / 2.0),
                        "qw": math.cos(yaw / 2.0),
                    }
                )
            waypoints.append(parsed)

        return waypoints

    def _start_callback(self, _request):
        if self.running or self.waiting_for_reach or self.send_pending:
            return TriggerResponse(success=False, message="mission is already running")

        if self.index >= len(self.waypoints):
            self.index = 0

        if not self._start_mission():
            return TriggerResponse(
                success=False, message="could not switch GBPlanner to waypoint mode"
            )

        self._schedule_send(0.0)
        return TriggerResponse(
            success=True,
            message="waypoint mission loaded; use Start Planner to begin motion",
        )

    def _start_mission(self):
        try:
            response = self.set_operation_mode(True)
        except rospy.ServiceException as error:
            rospy.logerr("Could not switch GBPlanner to waypoint mode: %s", error)
            return False
        if not response.success:
            rospy.logerr(
                "GBPlanner rejected waypoint mode: %s", response.message
            )
            return False

        # Every new mission requires a fresh operator Start Planner action.
        self.planner_started = False
        self.running = True
        rospy.loginfo(
            "GBPlanner waypoint mode enabled. The target will be loaded without "
            "starting PCI; use the Start Planner button to begin motion."
        )
        return True

    def _schedule_send(self, delay):
        if rospy.is_shutdown():
            return
        self.send_pending = True
        self.send_timer = rospy.Timer(
            rospy.Duration(delay), self._send_next_waypoint, oneshot=True
        )

    def _send_next_waypoint(self, _event):
        self.send_pending = False
        if not self.running or rospy.is_shutdown():
            return

        if self.index >= len(self.waypoints):
            self.running = False
            rospy.loginfo("Waypoint mission completed: all waypoints reached")
            return

        waypoint = self.waypoints[self.index]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = waypoint["x"]
        goal.pose.position.y = waypoint["y"]
        goal.pose.position.z = waypoint["z"]
        goal.pose.orientation.x = waypoint["qx"]
        goal.pose.orientation.y = waypoint["qy"]
        goal.pose.orientation.z = waypoint["qz"]
        goal.pose.orientation.w = waypoint["qw"]

        self.waiting_for_reach = True
        self.goal_publisher.publish(goal)
        if self.index > 0 and self.planner_started:
            # The target-reach tree can reset PCI when the previous goal is
            # completed. Re-trigger only after a real planner path proves that
            # the operator already pressed Start Planner for this mission.
            self.restart_timer = rospy.Timer(
                rospy.Duration(0.25), self._restart_planner, oneshot=True
            )
        rospy.loginfo(
            "Loaded GBPlanner target %d/%d '%s': (%.2f, %.2f, %.2f)",
            self.index + 1,
            len(self.waypoints),
            waypoint["name"],
            waypoint["x"],
            waypoint["y"],
            waypoint["z"],
        )

    def _path_callback(self, message):
        if self.running and message.poses:
            self.planner_started = True

    def _restart_planner(self, _event):
        if not self.running or rospy.is_shutdown():
            return
        try:
            response = self.start_planner()
        except rospy.ServiceException as error:
            rospy.logerr(
                "Could not continue planning to the next waypoint: %s. "
                "Use the Start Planner button to resume.",
                error,
            )
            return
        if not response.success:
            rospy.logerr(
                "PCI rejected continuation to the next waypoint: %s. "
                "Use the Start Planner button to resume.",
                response.message,
            )

    def _odometry_callback(self, message):
        self.position = message.pose.pose.position
        if not self.running or not self.waiting_for_reach:
            return

        waypoint = self.waypoints[self.index]
        dx = self.position.x - waypoint["x"]
        dy = self.position.y - waypoint["y"]
        dz = self.position.z - waypoint["z"]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > self.reached_distance:
            return

        self.waiting_for_reach = False
        rospy.loginfo(
            "Reached waypoint %d/%d '%s' (distance %.2f m)",
            self.index + 1,
            len(self.waypoints),
            waypoint["name"],
            distance,
        )
        self.index += 1
        self._schedule_send(self.inter_waypoint_delay)


def main():
    rospy.init_node("waypoint_follower")
    try:
        WaypointFollower()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Invalid waypoint follower configuration: %s", error)
        rospy.signal_shutdown(str(error))
    except rospy.ROSInterruptException:
        # Normal when roslaunch is stopped while waiting for PCI to appear.
        return
    rospy.spin()


if __name__ == "__main__":
    main()
