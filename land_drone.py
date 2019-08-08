from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

connection_string = '/dev/ttyACM0'
vehicle=0  

def connect2Drone():
    global vehicle
    # Connect to the Vehicle
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    print("connected")

            
            
def main():
    connect2Drone()
    print(vehicle.mode)
    vehicle.mode = VehicleMode("LAND")
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.close()

    print("Completed")
if __name__ == "__main__":
	main()
	