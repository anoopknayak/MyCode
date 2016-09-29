#!/usr/bin/env python

import file_access
import rospy

LOCATIONS_FILE_NAME = "savedLocations.txt"
TOURS_FILE_NAME = "tours.txt"


class Locations:
    locations = None

    def __init__(self):
        if self.locations is None:
            self.locations = file_access.read_as_dictionary(
                LOCATIONS_FILE_NAME)
            print "new loc"

        else:
            print "copy loc"

    def add(self, location_name, coordinates, can_update=False):
        if location_name in self.locations and not can_update:
            rospy.loginfo("Location name already exists")
            return False

        self.locations[location_name] = coordinates
        return True

    def remove(self, location_name):
        if location_name in self.locations:
            self.locations.pop(location_name)
            return True

        return False

    def read_coordinates(self, location_name):
        if location_name in self.locations:
            return self.locations[location_name]

        return None

    def write_locations_to_file(self):
        file_access.write_dictionary(self.locations, LOCATIONS_FILE_NAME)


class Tours:
    tours = None

    def __init__(self):
        if self.tours is None:
            self.tours = file_access.read_as_dictionary(TOURS_FILE_NAME)
            print "new tour"

        else:
            print "copy tour"

    def add_locations(self, name, locations_list, index=None):
        if name in self.tours:
            if index is not None and len(self.tours[name]) >= index:
                temp = self.tours[name][:index] + locations_list + \
                       self.tours[name][index:]
                self.tours[name] = temp

            else:
                self.tours[name].extend(locations_list)

        else:
            self.tours[name] = locations_list

    def read_locations(self, tour_name):
        if tour_name in self.tours:
            return self.tours[tour_name]

        return None

    def remove_location(self, tour_name, location_name):
        if tour_name in self.tours:
            try:
                self.tours[tour_name].remove(location_name)
                return True

            except ValueError:
                pass

        return False

    def write_tours_to_file(self):
        file_access.write_dictionary(self.tours, TOURS_FILE_NAME)
