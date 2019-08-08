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

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    global vehicle
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.3 #0.7
    SMOOTH_TAKEOFF_THRUST = 0.6
        
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    #while not vehicle.is_armable:
     #   print(" Waiting for vehicle to initialise...")
      #  time.sleep(1)

    print("is armable=",vehicle.is_armable)
    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    print(vehicle.armed)
    while not vehicle.armed:
	print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    for i in range(1000):
    #while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %s" % current_altitude)
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)
        
def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
                                                             0,
                                                             0,
                                                                 # Target system
                                                             0,
                                                                 # Target component
                                                             0b00000000,
                                                                 # Type mask: bit 1 is LSB
                                                             to_quaternion(roll_angle, pitch_angle),
                                                                 # Quaternion
                                                             0,
                                                                 # Body roll rate in radian
                                                             0,
                                                                 # Body pitch rate in radian
                                                             math.radians(yaw_rate),
                                                                 # Body yaw rate in radian
                                                             thrust)
                                                                 # Thrust
    vehicle.send_mavlink(msg)
                                                             
    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)
        
        # Sleep for the fractional part
        time.sleep(modf[0])
        
        # Send command to vehicle on 1 Hz cycle
        for x in range(0,int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)
            
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]

            
            
def main():
    connect2Drone()
    
    # Take off 2.5m in GUIDED_NOGPS mode.
    arm_and_takeoff_nogps(2.5)
    #quit()	
    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    print("Completed")
if __name__ == "__main__":
	main()
	