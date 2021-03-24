from tkinter import *
import cv2

import PickOrderLogic


class UserInterface:
    def __init__(self):
        #cv2.namedWindow('Dempers')
        #mode_switch = '0 : MANUAL \n1 : AUTO'
        #cv2.createTrackbar(mode_switch, 'Dempers', 0, 1, self.mode_switch)

        # Create classes
        pick_order_logic = PickOrderLogic.PickOrderLogic()

        pick_order_logic.start_automatic_mode()

    def mode_switch(self, x):
        pass
