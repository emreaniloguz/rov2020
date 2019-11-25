import cv2
import numpy as np

class Detection():
    def __init__(self):
        self.__min_red1 = np.array([0,70,50])
        self.__max_red1 = np.array([10,250,250])
        self.__min_red2 = np.array([170,70,50])
        self.__max_red2 = np.array([180,250,250])
        self.desired_area = None

    def operations(self,frame):
        #frame = cv2.resize(frame, (0,0), fx=0.25, fy=0.25)

        (self.height,self.width,_) = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(hsv, self.__min_red1, self.__max_red1)
        red_mask2 = cv2.inRange(hsv, self.__min_red2, self.__max_red2)
        self.red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        self.masked_frame = cv2.bitwise_and(frame, frame, mask=self.red_mask)

    def red_contour(self):
        _, contours, hierarchy = cv2.findContours(self.red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.cnt = contours[1]

    def area_error(self):
        area = cv2.contourArea(self.cnt)
        return self.desired_area - area

    def center_error(self):
        M = cv2.moments(self.cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        xCenter = int(self.width / 2)
        yCenter = int(self.height / 2)
        self.xTolerance = int(self.width/30)
        self.yTolerance = int(self.height/20)
        x_error = cx - xCenter
        y_error = cy - yCenter

        return {"x": x_error, "y": y_error}

    def warp_ratio(self):
        lines = cv2.HoughLinesP(self.red_mask, 1, np.pi / 180, 50, None, 50, 10)
        pass

