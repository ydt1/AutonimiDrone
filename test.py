import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import threading
import logging

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
from simple_pid import PID


connection_string = '/dev/ttyACM0'
vehicle=0
SETTINGS_FILE = "RTIMULib"
r,p,y = 0.0,0.0,0.0


def wait_for_calibrate():
    global r
    global p
    global y
    while (r == 0.0 and p == 0.0 and y == 0.0):
        logging.info("still calibrate %.1f, %.1f, %.1f", r,p,y)
        time.sleep(0.1)
        
def getIMU_DATA(name):
    global r
    global p
    global y
    logging.info("Thread %s: starting", name)
    logging.info("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
      print("Settings file does not exist, will be created")

    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    logging.info("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        logging.error("IMU Init Failed")
        sys.exit(1)
    else:
        logging.info("IMU Init Succeeded");

    # this is a good time to set any fusion parameters

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    logging.info("Recommended Poll Interval: %dmS\n", poll_interval)
    #geroR,geroP,geroY=143.24, -13.9,-25.4
    geroR,geroP,geroY=0.0,0.0,0.0
    
    ################## calibrate
    sampleCount = 0
    geroR=0
    geroP=0
    geroY=0
    logging.info("calibrating...")
    for i in range(3000):
	  if imu.IMURead():
		data = imu.getIMUData()
		fusionPose = data["fusionPose"]
		geroR += truncFloat(math.degrees(fusionPose[0]))
		geroP += truncFloat(math.degrees(fusionPose[1]))
		geroY += truncFloat(math.degrees(fusionPose[2]))
		sampleCount+=1;
		logging.debug("r: %f p: %f y: %f",truncFloat(math.degrees(fusionPose[0])), 
			truncFloat(math.degrees(fusionPose[1])), truncFloat(math.degrees(fusionPose[2])))
		time.sleep(poll_interval*1.0/1000.0)
    
    geroR = truncFloat(geroR/sampleCount)
    geroP = truncFloat(geroP/sampleCount)
    geroY = truncFloat(geroY/sampleCount)
    logging.info ("sample count= %d",sampleCount)
    logging.info ("Roll offset= %.1f",geroR)
    logging.info ("Pitch offset= %.1f",geroP)
    logging.info ("Yaw offset= %.1f",geroY)
    logging.info ("*************** Done calibrating ***************")
   
    ##################
    while True:	
        if imu.IMURead():
            # x, y, z = imu.getFusionData()
            # print("%f %f %f" % (x,y,z))
            data = imu.getIMUData()
            fusionPose = data["fusionPose"]
            #print("r: %.1f p: %.1f y: %.1f" % (truncFloat(math.degrees(fusionPose[0])-geroR), 
            #    truncFloat(math.degrees(fusionPose[1])-geroP), truncFloat(math.degrees(fusionPose[2])-geroY)))
            r,p,y = truncFloat(math.degrees(fusionPose[0])-geroR)* -1, truncFloat(math.degrees(fusionPose[1])-geroP), truncFloat(math.degrees(fusionPose[2])-geroY)
            if (r < -180):
                r = r + 360
            time.sleep(poll_interval*1.0/1000.0)
            
    

def truncFloat(f):
    return float('%.2f'%f)
    
def connect2Drone():
    global vehicle
    # Connect to the Vehicle
    logging.info('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    logging.info("connected")
    print(" Attitude: %s" % vehicle.attitude)
    print(" Battery: %s" % vehicle.battery)

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """
    global vehicle
    pidR = PID(0.4, 0, 0.05, setpoint=0)
    pidP = PID(0.4, 0, 0.05, setpoint=0)
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.55 #0.7
    SMOOTH_TAKEOFF_THRUST = 0.55
    wait_for_calibrate()    
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
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        #logging.info(" Altitude: %s" % current_altitude)
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            logging.info("Reached target altitude")
            set_attitude(thrust = 0.5) 
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        
        #controlR = pidR(r)
        #controlP = pidP(p)
       
        if (r<0.040 and r > -0.040):
            logging.info ("r between -0.040 to 0.040")
        elif (r<0):
            controlR = 1
        else:
            controlR = -1
        
        if (controlPCount % 8 ==0):       
            if (p<0.02):
                controlP = 3.2
            else:
                controlP = -0.5       
        else:
            controlPCount = controlPCount + 1
            
        logging.info ("r: %.1f p: %.1f y: %.1f a: %.1f", r,p,y,current_altitude)
        logging.info("controlP: %.1f controlR %.1f", controlP,controlR)
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
    
    connect2Drone()
    
    logging.info("\nAdd `attitude` attribute callback/observer on `vehicle`")     
    vehicle.add_attribute_listener('attitude', attitude_callback)
    #time.sleep(1000)
    #quit()
    
    
    #logging.info("Main    : before creating thread")
    #x = threading.Thread(target=getIMU_DATA, args=(1,))
    #logging.info("Main    : before running thread")
    #x.daemon = True
    #x.start()
    #logging.info("Main    : wait for the thread to finish")
    # Take off 2.5m in GUIDED_NOGPS mode.
    arm_and_takeoff_nogps(1.3)
    logging.info("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    #time.sleep(1)

    #x.join()
    logging.info("Main    : all done")    
        

    #quit()	
    
    # Uncomment the lines below for testing roll angle and yaw rate.
    # Make sure that there is enough space for testing this.

    # set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
    # set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)

    # Move the drone forward and backward.
    # Note that it will be in front of original position due to inertia.
    #print("Move forward")
    #set_attitude(pitch_angle = 1, thrust = 0.5, duration = 3.21)

    #print("Move backward")
    #set_attitude(pitch_angle = -1, thrust = 0.5, duration = 3)    
    
    #time.sleep(10000)
    
    
    
    
    # Close vehicle object before exiting script
    logging.info("Close vehicle object")
    vehicle.close()

    logging.info("Completed")
if __name__ == "__main__":
	main()
	
