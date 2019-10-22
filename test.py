import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import threading
import logging
import RPi.GPIO as GPIO

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from simple_pid import PID


connection_string = '/dev/ttyACM0'
vehicle=0
SETTINGS_FILE = "RTIMULib"
current_altitude,r,p,y = 0.0,0.0,0.0,0.0

# set goip for hte ultrasonic sensor
GPIO.setmode(GPIO.BOARD)
GPIO_TRIGGER = 33
GPIO_ECHO = 35

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)



def wait_for_calibrate():
    global r
    global p
    global y
    while (r == 0.0 and p == 0.0 and y == 0.0):
        logging.info("still calibrate %.1f, %.1f, %.1f", r,p,y)
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
        
        if abs(current_altitude-distance)>0.5 or distance > 5: # sometimes the ultrasonic sends bad high values - fliter them out
            logging.info("distance too high, might be bad data %.1f", distance)
        else:
            current_altitude=distance
        time.sleep(0.2)    
       




def truncFloat(f):
    return float('%.2f'%f)
    
def connect2Drone():
    global vehicle
    # Connect to the Vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("connected")


def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    global vehicle
    DEFAULT_TAKEOFF_THRUST = 0.55 #0.7
    SMOOTH_TAKEOFF_THRUST = 0.55
    #wait_for_calibrate()    
    logging.info("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check, just comment it with your own responsibility.
    #while not vehicle.is_armable:
     #   logging.info(" Waiting for vehicle to initialise...")
      #  time.sleep(1)

    logging.info("is armable=%s",vehicle.is_armable)
    logging.info("Arming motors")
    
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    
    while not vehicle.armed:
	logging.info(" Waiting for arming...")
        time.sleep(1)
    logging.info(vehicle.armed)
    logging.info("Taking off!")
    controlR = 0
    controlP = 0 
    controlPCount = 8
    controlRCount = 8 
    CountThrust = 0
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        #current_altitude = vehicle.location.global_relative_frame.alt #take from the pix not accurate for us, will use ultrasonic
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            logging.info("Reached target altitude")
            set_attitude(thrust = 0.5) 
            break
        elif current_altitude >= aTargetAltitude*0.6:
            if CountThrust < 3:
                thrust = DEFAULT_TAKEOFF_THRUST
            else:    
                thrust = SMOOTH_TAKEOFF_THRUST
        CountThrust+=1
        
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
            
        logging.info ("r: %.1f p: %.1f y: %.1f a: %.1f", r,p,y,current_altitude)
        logging.info("controlP: %.1f controlR %.1f thrust %.1f", controlP,controlR, thrust)
         # check the we don't get -1 on all values
        set_attitude(roll_angle = controlR, pitch_angle = controlP, thrust = thrust)
        
        time.sleep(0.1)
        
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
        #print(" CALLBACK: Attitude changed to", value.pitch)
        logging.info ("r: %.3f p: %.3f y: %.1f a: %.1f", value.roll,value.pitch,value.yaw,vehicle.location.global_relative_frame.alt)
        r,p,y = value.roll,value.pitch,value.yaw
        last_attitude_cache=value

     
def main():
    
    
    #logging.basicConfig(format='%(asctime)s %(message)s',level=logging.DEBUG,filename='test.log')
    logging.basicConfig(format='%(asctime)s %(message)s',level=logging.DEBUG)
    logging.info("\nAdd `attitude` attribute callback/observer on `vehicle`")  
    connect2Drone()
    vehicle.add_attribute_listener('attitude', attitude_callback)
    try:
        logging.info("Main    : before creating thread")
        x = threading.Thread(target=get_distance, args=(1,))
        logging.info("Main    : before running thread")
        x.daemon = True
        x.start()
        logging.info("Main    : wait for the thread to finish")   
        #Take off 2.5m in GUIDED_NOGPS mode.
        arm_and_takeoff_nogps(1.3)  
        #x.join()
        logging.info("Main    : all done") 
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup() 
    logging.info("Completed")
    logging.info("Close vehicle object")
    vehicle.close()

    logging.info("Completed")
if __name__ == "__main__":
	main()
	
