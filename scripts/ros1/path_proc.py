#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

path_publisher = rospy.Publisher('/command/path', Path, queue_size=1)
proc_ref_path_pub = rospy.Publisher('/proc/vis/ref_path', MarkerArray, queue_size=1)
goal_dir_pub = rospy.Publisher('/goal_dir', Marker, queue_size=1)

def path_callback(path_msg):
    length = 0.0
    prev_pt = path_msg.poses[0].pose.position
    for i in range(1, len(path_msg.poses) - 1):
        p1 = prev_pt
        p2 = path_msg.poses[i].pose.position
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        segment_length = (dx**2 + dy**2 + dz**2)**0.5
        if segment_length >= 0.02:
            length += segment_length
            prev_pt = p2

    rospy.loginfo("Length of the path: %.2f meters", length)

def trajectory_callback(trajectory_msg):
    path_msg = Path()
    path_msg.header = trajectory_msg.header

    for point in trajectory_msg.points:
        pose = PoseStamped()
        pose.header = trajectory_msg.header
        pose.pose.position.x = point.transforms[0].translation.x
        pose.pose.position.y = point.transforms[0].translation.y
        pose.pose.position.z = point.transforms[0].translation.z
        pose.pose.orientation = point.transforms[0].rotation

        path_msg.poses.append(pose)

    path_publisher.publish(path_msg)

# def ref_path_callback(ref_path):
#     t = rospy.Time.now().to_sec()
#     # if(t > 1756294165.10 + 740 and t < 1756294165.10 + 800): # lokken anymal
#     if(t > 1758906212.57 + 690 and t < 1758906212.57 + 730):
#         proc_ref_path = MarkerArray()

#         ref_path.markers[0].scale.x = 0.25
#         ref_path.markers[1].scale.x = 0.1
#         ref_path.markers[1].scale.y = 0.1
#         ref_path.markers[1].scale.z = 0.1
#         ref_path.markers[0].color.r = 0.0
#         ref_path.markers[0].color.g = 1.0
#         ref_path.markers[0].color.b = 0.0
#         ref_path.markers[0].color.a = 0.7

#         # remove all points in markers[0] and markers[1] for which x is greater than -61
#         # ref_path.markers[0].points = [p for p in ref_path.markers[0].points if p.x <= -61]
#         # ref_path.markers[1].points = [p for p in ref_path.markers[1].points if p.x <= -61]

#         # If markers[0] has odd number of points, remove the last point
#         if len(ref_path.markers[0].points) % 2 == 1:
#             ref_path.markers[0].points.pop()

#         print("Ref path cb, time: ", t, "num points in marker 0: ", len(ref_path.markers[0].points), "num points in marker 1: ", len(ref_path.markers[1].points))
        
#         # move all points in markers[0] down by 0.15 in z
#         # for p in ref_path.markers[0].points:
#         #     p.z -= 0.1
#         proc_ref_path.markers.append(ref_path.markers[0])
#         proc_ref_path.markers.append(ref_path.markers[1])

#         proc_ref_path_pub.publish(proc_ref_path)
def ref_path_callback(ref_path):
    proc_ref_path = MarkerArray()

    ref_path.markers[0].scale.x = 0.3
    ref_path.markers[1].scale.x = 0.3
    ref_path.markers[1].scale.y = 0.3
    ref_path.markers[1].scale.z = 0.3

    ref_path.markers[0].color.r = 1.0
    ref_path.markers[0].color.g = 234.0/255.0
    ref_path.markers[0].color.b = 0.0

    proc_ref_path.markers.append(ref_path.markers[0])
    proc_ref_path.markers.append(ref_path.markers[1])

    proc_ref_path_pub.publish(proc_ref_path)

def odom_callback(odom):
    # x: current position, y: current position, z: 0 (2D)

    # Fixed goal point
    # RL
    # goal_x = -40.0
    # goal_y = 3.5
    # goal_z = 0.9
    # NMPC
    goal_x = 40.0
    goal_y = 0.0
    goal_z = 0.8

    # Current position
    cur_x = odom.pose.pose.position.x
    cur_y = odom.pose.pose.position.y
    cur_z = odom.pose.pose.position.z

    marker = Marker()
    marker.header.frame_id = "mimosa_world"
    marker.header.stamp = odom.header.stamp
    marker.ns = "goal_direction"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    # Arrow defined by two points: tail -> head
    start = Point(cur_x, cur_y, cur_z)
    end   = Point(goal_x, goal_y, goal_z)
    marker.points = [start, end]

    # Scale: x = shaft diameter, y = head diameter, z = head length
    marker.scale.x = 0.15
    marker.scale.y = 0.2
    marker.scale.z = 0.3

    # Color 
    marker.color.r = 0.0
    marker.color.g = 0.5
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0)  # 0 = forever

    goal_dir_pub.publish(marker)

def path_length_calculator():
    rospy.init_node('path_length_calculator', anonymous=True)
    rospy.Subscriber('/mimosa_node/graph/path', Path, path_callback)
    rospy.Subscriber('/mimosa_node/graph/odometry', Odometry, odom_callback)
    # rospy.Subscriber('/command/trajectory', MultiDOFJointTrajectory, trajectory_callback)
    # rospy.Subscriber('/rmf_obelix/command/trajectory', MultiDOFJointTrajectory, trajectory_callback)
    rospy.Subscriber('/vis/ref_path', MarkerArray, ref_path_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        path_length_calculator()
    except rospy.ROSInterruptException:
        pass
