#! /usr/bin/env python
# -*- coding: utf-8 -*-

from math import degrees

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from tour_manager.srv import *

import points_of_interest as poi


class SaveLocation:
    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.location_received = False

        rospy.init_node("save_location_server")
        rospy.on_shutdown(poi.Locations.write_locations_to_file)

        rospy.Service('save_location', stdService, self.save_location)

        # Read the current pose topic
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                         self.callback_pose)

        rospy.spin()

    def callback_pose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w])

        self.yaw = degrees(self.yaw)
        # rospy.loginfo("Current robot pose: x=" + str(self.x) + " y=" + str(
        #     self.y) + " yaw=" + str(self.yaw) + "º")
        self.location_received = True

    def save_location(self, req):

        while not self.location_received:
            pass

        coordinates = [str(self.x), str(self.y), str(self.yaw)]

        return stdServiceResponse(poi.Locations.add(req.data, coordinates))


if __name__ == '__main__':

    SaveLocation()
