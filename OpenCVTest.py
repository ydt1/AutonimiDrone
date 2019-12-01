# install opencv & numpy

import cv2
import numpy as np
from PIL import Image
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from SocketServer import ThreadingMixIn
import StringIO

cap=None
FRAME_WIDTH=320
FRAME_HEIGHT=240

def nothing(x):
    pass

def setupCaptureObject():
    global cap
    cap = cv2.VideoCapture(0)# select the roght webcam to use (on a laptop 0=built-in webcam, 1=USB webcam)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # disable auto focus
    # if using LED lights than disable exposure and brightness
    cap.set(cv2.CAP_PROP_EXPOSURE, -100)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, -1)
    cap.set(cv2.CAP_PROP_FPS, 30)  # set FPS 30

def setupHSVTrackBars():
    cv2.namedWindow('Trackbar')
    cv2.createTrackbar('H Min', 'Trackbar', 58, 255, nothing)
    cv2.createTrackbar('H Max', 'Trackbar', 97, 255, nothing)

    cv2.createTrackbar('S Min', 'Trackbar', 13, 255, nothing)
    cv2.createTrackbar('S Max', 'Trackbar', 230, 255, nothing)

    cv2.createTrackbar('V Min', 'Trackbar', 105, 255, nothing)
    cv2.createTrackbar('V Max', 'Trackbar', 255, 255, nothing)

def readFromCam():
    kernel = np.ones((5, 5), np.uint8)

    while True:
        ret, img = cap.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of HSV
        HSV_MIN = np.array([cv2.getTrackbarPos('H Min', 'Trackbar'), cv2.getTrackbarPos(
            'S Min', 'Trackbar'), cv2.getTrackbarPos('V Min', 'Trackbar')])
        HSV_MAX = np.array([cv2.getTrackbarPos('H Max', 'Trackbar'), cv2.getTrackbarPos(
            'S Max', 'Trackbar'), cv2.getTrackbarPos('V Max', 'Trackbar')])

        # Threshold the HSV image what was defined in the trackbars
        threshold = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
        #erosion = cv2.erode(threshold, kernel, iterations=1)
        dilation = cv2.dilate(threshold, kernel, iterations=1)
        opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        # for contour in contours:
        #     cv2.drawContours(img, contour, -1, (0, 255, 0), 3)

        #closing = hsv
        contours, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0 :
            cv2.drawContours(img, contours, -1, (0,255,0), 3)

            cnt = contours[0]
            
            M = cv2.moments(cnt)

            cx = int(FRAME_WIDTH/2 - M['m10']/M['m00'])
            cy = int(FRAME_HEIGHT/2 - M['m01']/M['m00'])
            
            print(cx, cy)

        cv2.imshow("image", img)
        cv2.imshow("threshold", closing)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def main():
    setupCaptureObject()
    setupHSVTrackBars()
    readFromCam()

    # done reading from Camera, close all windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
