#!/usr/bin/env python

import rospy
import points_of_interest as poi

from tour_manager.srv import *


def handle_request(req):

    result = True

    tour_locations = poi.Tours.read_locations(req.data)

    if tour_locations is not None:

        for location_name in tour_locations:

            rospy.wait_for_service('navigate_to')

            try:
                poi_service = rospy.ServiceProxy('navigate_to', stdService)
                response = poi_service(location_name)
                if response.result:
                    rospy.loginfo("Successfully reached " + location_name)

                else:
                    rospy.loginfo(
                        "Unsuccessfully: unable to reach " + location_name)
                    result = False
                    break

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
                result = False

    else:
        rospy.loginfo("Tour does not exist")
        result = False

    return stdServiceResponse(result)


if __name__ == "__main__":

    rospy.init_node('do_tour_server')

    rospy.Service('do_tour', stdService, handle_request)
    rospy.spin()
