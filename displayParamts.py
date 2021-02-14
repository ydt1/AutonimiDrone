import sys

sys.path.append('.')

import os.path
import time
import math
import threading
import logging
from logging.handlers import RotatingFileHandler
import pyrealsense2.pyrealsense2 as rs


from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions



connection_string = '/dev/ttyACM0'
vehicle=0
SETTINGS_FILE = "RTIMULib"
current_altitude,r,p,y = 0.0,0.0,0.0,0.0

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)


def connect2Drone():
    global vehicle
    
    # Connect to the Vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("connected")

def getPoseRs2():
    frame = pipe.wait_for_frames()

    # Fetch pose frame
    pose = frame.get_pose_frame()
    if pose:
        # Print some of the pose data to the terminal
        data = pose.get_pose_data()
        # logging.info("Frame #{}".format(pose.frame_number))
        logging.info("Position: {}".format(data.translation))
        logging.info("Velocity: {}".format(data.velocity))
        logging.info("Acceleration: {}".format(data.acceleration))


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
        #print(" CALLBACK: Attitude changed to", value.pitch)
        #logging.info ("r: %.3f p: %.3f y: %.1f a: %.1f", value.roll,value.pitch,value.yaw,vehicle.location.global_relative_frame.alt)
        r,p,y = value.roll,value.pitch,value.yaw
        last_attitude_cache=value
        
        
# aTargetAltitude in CM        
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    global vehicle
    controlR = 0
    controlP = 0 
    controlA = 0 
    controlPCount = 8
    controlRCount = 8 
    controlACount = 8
    while True:
        #logging.info(vehicle.battery)
        #logging.info(vehicle.rangefinder)# will use the Lidar value and not the barometer one (vehicle.location.global_relative_frame.alt)
        alt = 100 * vehicle.rangefinder.distance
        #logging.info(alt)
        if alt >= aTargetAltitude*0.95: # Trigger just below target alt.
            logging.info("Reached target altitude")
            break

        # every 8 cycles we update the controlR/P values which are the degrees we send to the PIX    
        if (controlRCount % 8 ==0):     
            controlR = 0
            if (r<0.050 and r > -0.050):
                
                #logging.info ("r between -0.040 to 0.040")
                pass
            elif (r<0):
                controlR = 1
            else:
                controlR = -1
        else:
            controlRCount += 1
        
        if (controlPCount % 8 ==0):       
            controlP = 0 
            if (p<0.050 and p > -0.050):
                #logging.info ("p between -0.040 to 0.040")
                pass
            elif (p<0.02):
                controlP = 3.2
            else:
                controlP = -0.5       
        else:
            controlPCount = controlPCount + 1
           
        #control the alt in do action
        if (controlACount % 8 == 0):       
            controlA=0
            if (alt < 150):
                pass
                #logging.debug ("A between -0.040 to 0.040")
            else:
                controlA = 0.6 # Need to low the trust to go down in height(need to play with the trust)    
        else:
            controlACount += 1    
          
        getPoseRs2()
        logging.info ("r: %.3f p: %.3f y: %.1f a: %.3f controlP: %.1f controlR %.1f battery: %.1f\n", r,p,y,alt, controlP,controlR,vehicle.battery.voltage)
        
        time.sleep(0.05)
        


def setupLogging():
	logFormatter = logging.Formatter("%(asctime)s [%(threadName)-12.12s] %(message)s")
	rootLogger = logging.getLogger()
	rootLogger.setLevel(logging.DEBUG)

	fileHandler = RotatingFileHandler("displayParameters.log",maxBytes=5000000,backupCount=2)
	fileHandler.setFormatter(logFormatter)
	rootLogger.addHandler(fileHandler)

	consoleHandler = logging.StreamHandler(sys.stdout)
	consoleHandler.setFormatter(logFormatter)
	rootLogger.addHandler(consoleHandler)

def main():
    setupLogging()
    logging.info("Starting")

    
    connect2Drone()
    vehicle.add_attribute_listener('attitude', attitude_callback)
    try:
        arm_and_takeoff_nogps(1000)   
    except KeyboardInterrupt:
        print("stopped by User")
         
    logging.info("Completed")
if __name__ == "__main__":
    main()
	