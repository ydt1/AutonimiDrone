import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

connection_string = '/dev/ttyACM0'

class cmd:
    def __init__(self):
        self.vehicle = 0

    def connect(self):
        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=True)
        print("connected")

    def set_groundspeed(self, gs):
        self.vehicle.groundspeed=gs

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)


    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation


    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def goto(self, dNorth, dEast):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        gotoFunction = self.vehicle.simple_goto
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        gotoFunction(targetLocation)

        # print "DEBUG: targetLocation: %s" % targetLocation
        # print "DEBUG: targetLocation: %s" % targetDistance

        while self.vehicle.mode.name == "GUIDED":  # Stop action if we are no longer in guided mode.
            # print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance <= targetDistance * 0.01:  # Just below target, in case of undershoot.
                print("Reached target")
                break
            time.sleep(2)

    def land(self):
        print("Setting LAND mode...")
        self.vehicle.mode = VehicleMode("LAND")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()
        print("Completed")