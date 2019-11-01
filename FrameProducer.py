import cv2
import numpy as np

cap = cv2.VideoCapture(1)

cap.set(3, 320)
cap.set(4, 240)

cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # disable auto focus
cap.set(15, -100)
cap.set(10, -1)

cap.set(5, 30)  # set FPS 30

cv2.namedWindow('Trackbar')


def nothing(x):
    pass


cv2.createTrackbar('H Min', 'Trackbar', 58, 255, nothing)
cv2.createTrackbar('H Max', 'Trackbar', 97, 255, nothing)

cv2.createTrackbar('S Min', 'Trackbar', 13, 255, nothing)
cv2.createTrackbar('S Max', 'Trackbar', 230, 255, nothing)

cv2.createTrackbar('V Min', 'Trackbar', 105, 255, nothing)
cv2.createTrackbar('V Max', 'Trackbar', 255, 255, nothing)


kernel = np.ones((5, 5), np.uint8)

while True:
    ret, img = cap.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of HSV
    HSV_MAX = np.array([cv2.getTrackbarPos('H Max', 'Trackbar'), cv2.getTrackbarPos(
        'S Max', 'Trackbar'), cv2.getTrackbarPos('V Max', 'Trackbar')])
    HSV_MIN = np.array([cv2.getTrackbarPos('H Min', 'Trackbar'), cv2.getTrackbarPos(
        'S Min', 'Trackbar'), cv2.getTrackbarPos('V Min', 'Trackbar')])

    # Threshold the HSV image to get only blue colors
    threshold = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
    #erosion = cv2.erode(threshold, kernel, iterations=1)
    dilation = cv2.dilate(threshold, kernel, iterations=1)
    opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # erosion = cv2.erode(threshold, kernel, iterations=1)
    # dilation = cv2.dilate(erosion, kernel, iterations=1)
    # closing = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)
    # opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)

    # contours = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # for contour in contours:
    #     cv2.drawContours(img, contour, -1, (0, 255, 0), 3)

    contours, _ = cv2.findContours(closing, 1, 2)
    if len(contours) > 0 :
        cv2.drawContours(img, contours, -1, (0,255,0), 3)

        cnt = contours[0]
        
        M = cv2.moments(cnt)

        cx = int(160 - M['m10']/M['m00'])
        cy = int(120 - M['m01']/M['m00'])
        
        print(cx, cy)

    cv2.imshow("image", img)
    cv2.imshow("threshold", closing)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
