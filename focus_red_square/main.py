from videoGet import Video
from squareDetect import Detection
from ..master.master import *

import cv2
from pymavlink import mavutil

video = Video(port=4777)
detection = Detection()

master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
master.wait_heartbeat()
master.arducopter_arm()

while True:

    if not video.frame_available():
        continue

    frame = video.frame()
    frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    detection.operations(frame)
    detection.red_contour()
    error_area = detection.area_error()
    error = detection.center_error()

    error_x = error["x"]
    error_y = error["y"]

    if error_area > 600:
        error_area = 600
    elif error_area > -600:
        error_area = -600

    if error_x > 600:
        error_x = 600
    elif error_x < -600:
        error_x = -600

    if error_y > 600:
        error_y = 600
    elif error_y < -600:
        error_y = -600

    print("error_X: ", error_x)
    print("error_y: ", error_y)
    print("error_area: ", error_area)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
