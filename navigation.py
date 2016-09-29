#! /usr/bin/env python

"""
Created on Jan 10 11:16:00 2014

@author: Sam Pfeiffer

Snippet of code on how to send a navigation goal and how to get the current
robot position in map

Navigation actionserver: /move_base/goal
Type of message: move_base_msgs/MoveBaseActionGoal

Actual robot pose topic: /amcl_pose
Type of message: geometry_msgs/PoseWithCovarianceStamped

"""

from math import radians

import actionlib
import rospy
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from tour_manager.srv import *

import points_of_interest as poi


def handle_request(req):

    coordinates = poi.Locations.read_coordinates(req.data)

    if coordinates is not None:
        result = navigate_to(coordinates)

        if result == 3:
            rospy.loginfo("Destination Arrived")
            return stdServiceResponse(True)

        else:
            if result == 4:
                rospy.loginfo("Request Aborted: couldn't get there")

            else:
                rospy.loginfo("Request Rejected")

    else:
        rospy.loginfo("Location does not exist")

    return stdServiceResponse(False)


def create_nav_goal(x, y, yaw):
    """Create a MoveBaseGoal with x, y position and yaw rotation (in degrees).
    Returns a MoveBaseGoal"""
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = '/map'  # Note: the frame_id must be
    #  map
    mb_goal.target_pose.pose.position.x = x
    mb_goal.target_pose.pose.position.y = y
    mb_goal.target_pose.pose.position.z = 0.0  # z must be 0.0 (no height in
    # the map)

    # Orientation of the robot is expressed in the yaw value of euler angles
    angle = radians(yaw)  # angles are expressed in radians
    quat = quaternion_from_euler(0.0, 0.0, angle)  # roll, pitch, yaw
    mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())

    return mb_goal


def navigate_to(position):
    # Connect to the navigation action server
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo("Connecting to /move_base AS...")
    nav_as.wait_for_server()
    rospy.loginfo("Connected.")

    rospy.loginfo("Creating navigation goal...")
    nav_goal = create_nav_goal(float(position[0]), float(position[1]),
                               float(position[2]))
    rospy.loginfo("Sending goal...")
    nav_as.send_goal(nav_goal)
    rospy.loginfo("Waiting for result...")
    nav_as.wait_for_result()
    nav_state = nav_as.get_state()
    rospy.loginfo("Done!")

    return nav_state
    # 3 is SUCCESS, 4 is ABORTED (couldn't get there), 5 REJECTED (the goal
    # is not attainable)


if __name__ == '__main__':
    rospy.init_node('navigate_to_server')

    rospy.Service('navigate_to', stdService, handle_request)
    rospy.spin()
