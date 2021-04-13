import cv2
import numpy as np


class Blob:
    def __init__(self, h, w, x, y, centre_x, centre_y):
        self.state = False
        self.height = h
        self.width = w
        self.centre_x = centre_x
        self.centre_y = centre_y
        self.x = x
        self.y = y

    def switch_state(self):
        self.state = True
