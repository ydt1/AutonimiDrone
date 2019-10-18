#import the GPIO and time package
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO_TRIGGER = 33
GPIO_ECHO = 35
 
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def distance():
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
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    #print ("Measured Distance = %.1f cm" % distance)
    distance = float(distance / 100)
    return distance
 
if __name__ == '__main__':
    try:
        while True:
            dist = distance()
            print ("Measured Distance = %.3f m" % dist)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()        