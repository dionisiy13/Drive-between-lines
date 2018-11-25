import cv2
import numpy as np
from pprint import pprint

class ProcessFrame:

    top_border_color = []
    bottom_border_color = []

    def set_top_border_color(self, item):
        self.top_border_color = item

    def set_bottom_border_color(self, item):
        self.bottom_border_color = item

    def apply_hsv_color(self, frame):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    def get_masked_frame(self, frame):
        #pprint(self.top_border_color)
	return cv2.inRange(frame, self.bottom_border_color, self.top_border_color)

    def binaryzation(self, hsv_frame):
        #pprint(self.top_border_color)
	return cv2.inRange(hsv_frame, np.array(self.bottom_border_color), np.array(self.top_border_color))

