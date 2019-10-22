import sys, getopt

sys.path.append('.')

import os.path
import time
import math
import threading
import logging

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO_TRIGGER = 33
GPIO_ECHO = 35

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

connection_string = '/dev/ttyACM0'
vehicle=0
SETTINGS_FILE = "RTIMULib"
current_altitude,r,p,y = 0.0,0.0,0.0,0.0


def wait_for_calibrate():
    global r
    global p
    global y
    while (r == 0.0 and p == 0.0 and y == 0.0):
        logging.info("still calibrate %.1f, %.1f, %.1f", r,p,y)
        time.sleep(0.1)
def truncFloat(f):
    return float('%.2f'%f)
    
def connect2Drone():
    global vehicle
    # Connect to the Vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("connected")
   
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
        
        
        
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    
    global vehicle
    controlR = 0
    controlP = 0 
    controlPCount = 8
    controlRCount = 8 
    
    while True:
        
        #current_altitude = vehicle.location.global_relative_frame.alt #take from the pix not accurate for us, will use ultrasonic
        #logging.info(" Altitude: %s" % current_altitude)
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            logging.info("Reached target altitude")
            break
            
        if (controlRCount % 8 ==0):     
            if (r<0.040 and r > -0.040):
                logging.info ("r between -0.040 to 0.040")
            elif (r<0):
                controlR = 1
            else:
                controlR = -1
        else:
            controlRCount += 1
        if (controlPCount % 8 ==0):       
            if (p<0.040 and p > -0.040):
                logging.info ("p between -0.040 to 0.040")
            elif (p<0.02):
                controlP = 3.2
            else:
                controlP = -0.5       
        else:
            controlPCount = controlPCount + 1
            
        logging.info ("r: %.1f p: %.1f y: %.1f a: %.1f controlP: %.1f controlR %.1f", r,p,y,current_altitude, controlP,controlR)
       
        time.sleep(0.1)
        
def get_distance(name):
    global current_altitude
    #*********** don't put any print / logging from here *************
    logging.info("Starting get distance threading")
    while True:
        
        GPIO.output(GPIO_TRIGGER, True)
     
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
     
        StartTime = time.time()
        StopTime = time.time()
        
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
            
        # save time of arrival
        
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
            
        #*********** don't put any print / logging up to here *************
        
        
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        distance = float(distance / 100) # In meters...
        
        if abs(current_altitude-distance)>1 or distance > 5: # sometimes the ultrasonic sends bad high values - fliter them out
            logging.info("distance too high, might be bad data %.1f", distance)
        else:
            current_altitude=distance
        #logging.info(current_altitude)
        time.sleep(0.2)    


   
def main():
    
    
    #logging.basicConfig(format='%(asctime)s %(message)s',level=logging.DEBUG,filename='test.log')
    logging.basicConfig(format='%(asctime)s %(message)s',level=logging.DEBUG)

    connect2Drone()
    vehicle.add_attribute_listener('attitude', attitude_callback)
    try:
        logging.info("Main    : before creating thread")
        x = threading.Thread(target=get_distance, args=(1,))
        logging.info("Main    : before running thread")
        x.daemon = True
        x.start()
        logging.info("Main    : wait for the thread to finish")   
        #time.sleep(1000)
        #Take off 2.5m in GUIDED_NOGPS mode.
        arm_and_takeoff_nogps(1.3)   
        #x.join()
        logging.info("Main    : all done") 
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup() 
    logging.info("Completed")
if __name__ == "__main__":
	main()
	