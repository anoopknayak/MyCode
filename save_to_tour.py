#!/usr/bin/env python

import rospy
import points_of_interest as poi

from tour_manager.srv import *


def handle_request(req):

    poi.Tours.add_locations(req.tourName, req.locations)

    return tourResponse(True)


if __name__ == "__main__":

    rospy.init_node('save_tour_server')
    rospy.on_shutdown(poi.Tours.write_tours_to_file)

    rospy.Service('save_tour', tour, handle_request)
    rospy.spin()
