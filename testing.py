#!/usr/bin/env python

import sys

import rospy
from tour_manager.srv import *


def save_location_client(location_name):
    rospy.wait_for_service('save_location')

    try:
        poi_service = rospy.ServiceProxy('save_location', stdService)
        response = poi_service(location_name)
        if response.result:
            rospy.loginfo("Successfully saved location as " + location_name)

        else:
            rospy.loginfo(
                "Unsuccessful, could not save the location as " + location_name)

        return response.result

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def navigate_to_client(location_name):
    rospy.wait_for_service('navigate_to')

    try:
        navigate_to_service = rospy.ServiceProxy('navigate_to', stdService)
        response = navigate_to_service(location_name)

        if response.result:
            rospy.loginfo("Successfully navigated to " + location_name)

        else:
            rospy.loginfo("Unsuccessful, can not navigate to " + location_name)

        return response.result

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def save_tour_client(tour_name, locations):
    rospy.wait_for_service('save_tour')

    try:
        save_tour_service = rospy.ServiceProxy('save_tour', tour)
        response = save_tour_service(tour_name, locations)
        if response.result:
            rospy.loginfo("Successfully saved tour: " + tour_name)

        else:
            rospy.loginfo("Unsuccessful, could not save tour: " + tour_name)

        return response.result

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def do_tour_client(tour_name):
    rospy.wait_for_service('do_tour')

    try:
        do_tour_service = rospy.ServiceProxy('do_tour', stdService)
        response = do_tour_service(tour_name)
        if response.result:
            rospy.loginfo("Successfully completed tour: " + tour_name)

        else:
            rospy.loginfo("Unsuccessful, can not do tour: " + tour_name)

        return response.result

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def usage():
    print "%s [save/go-to] <location name> \n OR \n" % sys.argv[0]
    print "%s save-to <tour name> <location names> \n OR \n" % sys.argv[0]
    print "%s do <tour name> \n" % sys.argv[0]
    sys.exit(1)


if __name__ == "__main__":

    rospy.init_node("testing")

    if len(sys.argv) > 2:

        if sys.argv[1].lower() == "save":
            # location_name = ' '.join(word for word in sys.argv[2:])
            save_location_client(sys.argv[2])

        elif sys.argv[1].lower() == "go-to":
            # location_name = ' '.join(word for word in sys.argv[2:])
            navigate_to_client(sys.argv[2])

        elif sys.argv[1].lower() == "save-to":
            save_tour_client(sys.argv[2], sys.argv[3:])

        elif sys.argv[1].lower() == "do":
            do_tour_client(sys.argv[2])

        else:
            usage()

    else:
        usage()
