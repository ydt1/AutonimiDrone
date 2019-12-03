# install opencv & numpy

import cv2
import numpy as np
from PIL import Image
import threading
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from SocketServer import ThreadingMixIn
import StringIO
import time

cap=None
img=None
FRAME_WIDTH=320
FRAME_HEIGHT=240
CAM_ID=1 # on laptop 0 for built-in cam, 1 for USB
SHOW_WINDOWS=False


H_MIN=0
H_MAX=0
S_MIN=0
S_MAX=0
V_MIN=255
V_MAX=255

def nothing(x):
    pass

def setupCaptureObject():
    global cap
    cap = cv2.VideoCapture(CAM_ID)# select the right webcam to use (on a laptop 0=built-in webcam, 1=USB webcam)

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
    global img
    while True:
        ret, img = cap.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # define range of HSV
        if SHOW_WINDOWS:        
            HSV_MIN = np.array([cv2.getTrackbarPos('H Min', 'Trackbar'), cv2.getTrackbarPos(
                'S Min', 'Trackbar'), cv2.getTrackbarPos('V Min', 'Trackbar')])
            HSV_MAX = np.array([cv2.getTrackbarPos('H Max', 'Trackbar'), cv2.getTrackbarPos(
                'S Max', 'Trackbar'), cv2.getTrackbarPos('V Max', 'Trackbar')])
        else:
            HSV_MIN = np.array([H_MIN,S_MIN ,V_MIN ])
            HSV_MAX = np.array([H_MAX,S_MAX ,V_MAX ])

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

        if SHOW_WINDOWS:
            cv2.imshow("image", img)
            cv2.imshow("threshold", closing)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


class CamHandler(BaseHTTPRequestHandler):
	def do_GET(self):
		if self.path.endswith('.mjpg'):
			self.send_response(200)
			self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
			self.end_headers()
			while True:
				try:
					'''
                    rc,img = capture.read()
					if not rc:
						continue
					'''
					imgRGB=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
					jpg = Image.fromarray(imgRGB)
					tmpFile = StringIO.StringIO()
					jpg.save(tmpFile,'JPEG')
					
					self.wfile.write("--jpgboundary")
					self.send_header('Content-type','image/jpeg')
					self.send_header('Content-length',str(tmpFile.len))
					self.end_headers()
					
					self.wfile.write( tmpFile.getvalue() )#jpg.save(self.wfile,'JPEG')
					
					time.sleep(0.05)
				except KeyboardInterrupt:
					
					break
			return
		if self.path.endswith('.html'):
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			self.wfile.write('<html><head></head><body>')
			self.wfile.write('<img src="http://127.0.0.1:8080/cam.mjpg"/>')
			self.wfile.write('</body></html>')
			return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""




def main():
    setupCaptureObject()
    if SHOW_WINDOWS:
        setupHSVTrackBars()
        readFromCam()
    else:
        x = threading.Thread(target=readFromCam)
        x.start()    
        try:
            server = ThreadedHTTPServer(('', 8080), CamHandler)
            print "server started"
            server.serve_forever()
        except KeyboardInterrupt:
            print "about to exit.."
            cap.release()
            server.socket.close()
            quit()


    # done reading from Camera, close all windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
