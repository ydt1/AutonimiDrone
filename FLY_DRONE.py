'''
TODO:
1. add controlA se we stay on the same altitude after take off  

'''


import sys

sys.path.append('.')

import os.path
import time
import math
import threading
import logging
from logging.handlers import RotatingFileHandler

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions



connection_string = '/dev/ttyACM0'
vehicle=0
r,p,y = 0.0,0.0,0.0
controlR = 0
controlP = 0 
controlPCount = 8
controlRCount = 8
DEFAULT_TAKEOFF_THRUST = 0.65  #0.7
SMOOTH_TAKEOFF_THRUST = DEFAULT_TAKEOFF_THRUST - 0.1

def connect2Drone():
    global vehicle
    # Connect to the Vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("connected")

def getControlPcontrolR():
    global controlR 
    global controlP
    global controlPCount 
    global controlRCount 
	# every 8 cycles we update the controlR/P values which are the degrees we send to the PIX
    if (controlRCount % 8 ==0):     
        controlR=0
        if (r > -0.040 and  r<0.040):
            #logging.debug ("r between -0.040 to 0.040")
            pass
        elif (r<0):
            controlR = 1
        else:
            controlR = -1
    else:
        controlRCount += 1

    if (controlPCount % 8 ==0):       
        controlP=0
        if (p<0.040 and p > -0.040):
            pass
            #logging.debug ("p between -0.040 to 0.040")
        elif (p<0.02):
            controlP = 3.2
        else:
            controlP = -0.5       
    else:
        controlPCount = controlPCount + 1
    printDroneInfo()

# aTargetAltitude in CM         
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    global vehicle
    DEFAULT_TAKEOFF_THRUST
    SMOOTH_TAKEOFF_THRUST

    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
     
    
    logging.info("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    '''
    while not vehicle.is_armable:
        logging.info(" Waiting for vehicle to initialise...")
        time.sleep(1)
    '''
    logging.info("is armable=%s",vehicle.is_armable)
    logging.info("Arming motors")
    
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    
    while not vehicle.armed:
	    logging.info(" Waiting for arming...")
	    time.sleep(1)        
    
    logging.info("vehicle.armed="+str(vehicle.armed))
    logging.info("Taking off!!!") 
    
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        alt = 100 * vehicle.rangefinder.distance
        #current_altitude = vehicle.location.global_relative_frame.alt #take from the pix not accurate for us, will use ultrasonic
        if alt >= aTargetAltitude*0.75: # Trigger just below target alt.
            logging.info("Reached target altitude  ******************")
            set_attitude(thrust = 0.5) 
            break
        elif alt >= aTargetAltitude*0.6: # if we reached 60% of the altitude than slow down a little
            thrust = SMOOTH_TAKEOFF_THRUST

        getControlPcontrolR()    
        # check that we don't get -1 on all values
        set_attitude(roll_angle = controlR, pitch_angle = controlP, thrust = thrust)
        
        time.sleep(0.05) 

def printDroneInfo():
    alt = 100 * vehicle.rangefinder.distance
    logging.info ("r: %.3f p: %.3f y: %.1f a: %.3f controlP: %.1f controlR: %.1f battery: %s", r,p,y,alt, controlP,controlR,vehicle.battery)    

def doAction():
    logging.info("standing...")
    for i in range(2000):
        getControlPcontrolR()
        set_attitude(roll_angle = controlR, pitch_angle = controlP, yaw_rate = 0.0, thrust = SMOOTH_TAKEOFF_THRUST, duration = 0)
        time.sleep(0.001)
    

    logging.info("going left")
    for i in range(500):
        getControlPcontrolR()
        set_attitude(roll_angle = -3, pitch_angle = 10, yaw_rate = 0.0, thrust = SMOOTH_TAKEOFF_THRUST, duration = 0)
        time.sleep(0.001)
    logging.info("landing...")
    '''
    for i in range(3000):
        getControlPcontrolR()
        set_attitude(roll_angle = controlP, pitch_angle = controlP, yaw_rate = 0.0, thrust = 0.5, duration = 0)
        time.sleep(0.001)
    logging.info("going left")    
    for i in range(1000): 
        getControlPcontrolR()
        set_attitude(roll_angle = -5, pitch_angle = controlP, yaw_rate = 0.0, thrust = 0.5, duration = 0)
        time.sleep(0.001)
    logging.info("landing")
    '''
    vehicle.mode = VehicleMode("LAND")
    
def set_attitude(roll_angle = 0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
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

     
#Define callback for `vehicle.attitude` observer
last_attitude_cache = None
def attitude_callback(self, attr_name, value):
    global r
    global p
    global y
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `self` - the associated vehicle object (used if a callback is different for multiple vehicles)
    # `value` is the updated attribute value.
    global last_attitude_cache
    # Only publish when value changes
    if value!=last_attitude_cache:
        #logging.info ("r: %.3f p: %.3f y: %.1f a: %.1f", value.roll,value.pitch,value.yaw,vehicle.location.global_relative_frame.alt)
        r,p,y = value.roll,value.pitch,value.yaw
        last_attitude_cache=value

		
		
def setupLogging():
	logFormatter = logging.Formatter("%(asctime)s %(message)s")
	rootLogger = logging.getLogger()
	rootLogger.setLevel(logging.DEBUG)

	fileHandler = RotatingFileHandler("flyDrone.log",maxBytes=5000000,backupCount=2)
	fileHandler.setFormatter(logFormatter)
	rootLogger.addHandler(fileHandler)

	consoleHandler = logging.StreamHandler(sys.stdout)
	consoleHandler.setFormatter(logFormatter)
	rootLogger.addHandler(consoleHandler)

     
def main():
    
    setupLogging()    
    connect2Drone()
    vehicle.add_attribute_listener('attitude', attitude_callback)
    try:
          
        #Take off 100cm in GUIDED_NOGPS mode.  
        arm_and_takeoff_nogps(100)  
        doAction()
    except KeyboardInterrupt:
        logging.error("stopped by User")
        logging.info("landing")
        vehicle.mode = VehicleMode("LAND")         
        vehicle.close()
    logging.info("Completed")
    vehicle.close()
if __name__ == "__main__":
	main()
	