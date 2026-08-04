#!/usr/bin/env python3

"""Publish a direct start-to-goal path without doing collision checking."""

import copy
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class StraightLineGlobalPlanner:
    """Turn each goal into the two-point Path expected by the NMPC."""

    def __init__(self):
        self.odometry_topic = rospy.get_param("~odometry_topic", "/rmf/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/gbplanner_path")
        self.frame_id = rospy.get_param("~frame_id", "world")

        self._latest_odometry = None
        self._pending_goal = None

        # Latching prevents a path selected before the ROS 1/ROS 2 bridge or
        # NMPC subscriber is ready from being lost.
        self._path_publisher = rospy.Publisher(
            self.path_topic, Path, queue_size=1, latch=True
        )
        self._odometry_subscriber = rospy.Subscriber(
            self.odometry_topic, Odometry, self._odometry_callback, queue_size=1
        )
        self._goal_subscriber = rospy.Subscriber(
            self.goal_topic, PoseStamped, self._goal_callback, queue_size=1
        )

        rospy.loginfo(
            "Straight-line global planner ready: %s + %s -> %s",
            self.odometry_topic,
            self.goal_topic,
            self.path_topic,
        )

    @staticmethod
    def _position_is_finite(pose):
        position = pose.position
        return all(
            math.isfinite(value)
            for value in (position.x, position.y, position.z)
        )

    @staticmethod
    def _normalized_orientation(orientation):
        result = copy.deepcopy(orientation)
        norm = math.sqrt(
            result.x * result.x
            + result.y * result.y
            + result.z * result.z
            + result.w * result.w
        )
        if not math.isfinite(norm) or norm < 1e-9:
            result.x = 0.0
            result.y = 0.0
            result.z = 0.0
            result.w = 1.0
            return result

        result.x /= norm
        result.y /= norm
        result.z /= norm
        result.w /= norm
        return result

    def _odometry_callback(self, message):
        self._latest_odometry = message
        if self._pending_goal is not None:
            goal = self._pending_goal
            self._pending_goal = None
            self._publish_path(goal)

    def _goal_callback(self, message):
        if not self._position_is_finite(message.pose):
            rospy.logerr("Ignoring goal with non-finite coordinates")
            return

        if self._latest_odometry is None:
            self._pending_goal = message
            rospy.loginfo("Goal received; waiting for the first odometry message")
            return

        self._publish_path(message)

    def _publish_path(self, goal):
        start_pose = self._latest_odometry.pose.pose
        if not self._position_is_finite(start_pose):
            rospy.logerr("Cannot plan from odometry with non-finite coordinates")
            self._pending_goal = goal
            return

        stamp = rospy.Time.now()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id

        start = PoseStamped()
        start.header = path.header
        start.pose = copy.deepcopy(start_pose)
        start.pose.orientation = self._normalized_orientation(
            start.pose.orientation
        )

        target = PoseStamped()
        target.header = path.header
        target.pose = copy.deepcopy(goal.pose)
        target.pose.orientation = self._normalized_orientation(
            target.pose.orientation
        )

        path.poses = [start, target]
        self._path_publisher.publish(path)

        dx = target.pose.position.x - start.pose.position.x
        dy = target.pose.position.y - start.pose.position.y
        dz = target.pose.position.z - start.pose.position.z
        rospy.loginfo(
            "Published %.2f m straight path to (%.2f, %.2f, %.2f) on %s",
            math.sqrt(dx * dx + dy * dy + dz * dz),
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
            self.path_topic,
        )


def main():
    rospy.init_node("straight_line_global_planner")
    StraightLineGlobalPlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
