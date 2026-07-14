#!/usr/bin/env python3

"""Send a configured sequence of waypoints to GBPlanner PCI."""

import math

import rospy
from planner_msgs.srv import pci_to_waypoint, pci_to_waypointRequest
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse


class WaypointFollower:
    def __init__(self):
        self.waypoints = self._load_waypoints()
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.inter_waypoint_delay = max(
            0.0, float(rospy.get_param("~inter_waypoint_delay", 0.5))
        )
        self.retry_delay = max(
            0.0, float(rospy.get_param("~retry_delay", 2.0))
        )
        self.max_retries = max(0, int(rospy.get_param("~max_retries", 0)))
        self.waypoint_service_name = rospy.get_param(
            "~waypoint_service", "/pci_to_waypoint"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/gbplanner_status"
        )

        self.index = 0
        self.retry_count = 0
        self.running = False
        self.waiting_for_result = False
        self.send_pending = False
        self.send_timer = None

        self.status_subscriber = rospy.Subscriber(
            self.status_topic, Bool, self._status_callback, queue_size=10
        )
        self.start_server = rospy.Service("~start", Trigger, self._start_callback)

        rospy.loginfo(
            "Waypoint follower: waiting for service %s", self.waypoint_service_name
        )
        rospy.wait_for_service(self.waypoint_service_name)
        self.send_waypoint = rospy.ServiceProxy(
            self.waypoint_service_name, pci_to_waypoint, persistent=False
        )

        rospy.loginfo(
            "Waypoint follower ready with %d waypoint(s); start service is %s/start",
            len(self.waypoints),
            rospy.get_name(),
        )
        if rospy.get_param("~autostart", False):
            self._schedule_send(0.5)
            self.running = True

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
        if self.running or self.waiting_for_result or self.send_pending:
            return TriggerResponse(success=False, message="mission is already running")

        if self.index >= len(self.waypoints):
            self.index = 0
            self.retry_count = 0

        self.running = True
        self._schedule_send(0.0)
        return TriggerResponse(success=True, message="waypoint mission started")

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
        request = pci_to_waypointRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self.frame_id
        request.waypoint.position.x = waypoint["x"]
        request.waypoint.position.y = waypoint["y"]
        request.waypoint.position.z = waypoint["z"]
        request.waypoint.orientation.x = waypoint["qx"]
        request.waypoint.orientation.y = waypoint["qy"]
        request.waypoint.orientation.z = waypoint["qz"]
        request.waypoint.orientation.w = waypoint["qw"]

        # Set this before the service call so an immediately reached target is
        # not missed while the synchronous call is returning.
        self.waiting_for_result = True
        try:
            self.send_waypoint(request)
        except rospy.ServiceException as error:
            self.waiting_for_result = False
            self.running = False
            rospy.logerr("Could not send %s: %s", waypoint["name"], error)
            return

        rospy.loginfo(
            "Sent waypoint %d/%d '%s': (%.2f, %.2f, %.2f)",
            self.index + 1,
            len(self.waypoints),
            waypoint["name"],
            waypoint["x"],
            waypoint["y"],
            waypoint["z"],
        )

    def _status_callback(self, message):
        if not self.running or not self.waiting_for_result:
            return

        self.waiting_for_result = False
        waypoint = self.waypoints[self.index]
        if message.data:
            rospy.loginfo(
                "Reached waypoint %d/%d '%s'",
                self.index + 1,
                len(self.waypoints),
                waypoint["name"],
            )
            self.index += 1
            self.retry_count = 0
            self._schedule_send(self.inter_waypoint_delay)
            return

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            rospy.logwarn(
                "Waypoint '%s' failed; scheduling retry %d/%d",
                waypoint["name"],
                self.retry_count,
                self.max_retries,
            )
            self._schedule_send(self.retry_delay)
            return

        self.running = False
        rospy.logerr(
            "Waypoint mission stopped: planner failed at waypoint %d/%d '%s'",
            self.index + 1,
            len(self.waypoints),
            waypoint["name"],
        )


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
