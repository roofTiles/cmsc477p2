import queue
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from robomaster import robot

import cv2
import time
from robomaster import camera
import csv

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_chassis = ep_robot.chassis

def StrafeToPartner():
    not_seeing_partner = True
    y_val = 0.2
    while not_seeing_partner:
        
        ep_chassis.move(x=0, y=y_val, z=0, xy_speed=0.7).wait_for_completed()
        # Check if the detected robot bounding box is centered, if so, not_seeing_partner is false

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    y_val = 0.2

    StrafeToPartner()

    ep_robot.close()

