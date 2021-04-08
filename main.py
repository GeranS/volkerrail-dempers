# VolkerRail dempers
# First version:        2021-03-22
# First test:           2021-03-24
# First successful run: 2021-04-01

from RailDetection import RailCamera
from PickOrderLogic import PickOrderLogic

# todo: robot script needs to be updated after PLC communication is implemented
# https://drive.google.com/drive/u/2/folders/1ibVbZg63A4amtCqi1D6-uhpEtu5gF_xN

print("Starting...")
logic = PickOrderLogic()

logic.start_testing_mode()

#RailCamera.RailCamera()
