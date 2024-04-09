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

def GrabLego(ep_gripper=None, ep_arm=None):
    # open gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()

    # Move forward 20mm
    ep_arm.move(x=20, y=-5).wait_for_completed()

    # close gripper
    ep_gripper.close(power=50)
    time.sleep(1)
    ep_gripper.pause()

    # Move backward 20mm
    ep_arm.move(x=-20, y=5).wait_for_completed()

def DropLego(ep_gripper=None, ep_arm=None):
    # Move forward 20mm
    ep_arm.move(x=20, y=-5).wait_for_completed()

    # close gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()

    # Move backward 20mm
    ep_arm.move(x=-20, y=5).wait_for_completed()

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gripper = ep_robot.gripper

    ep_arm = ep_robot.robotic_arm

    GrabLego()

    ep_robot.close()

