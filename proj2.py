import queue
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from robomaster import robot

import cv2
import time
from robomaster import camera
import csv
import gripping

def search():
    ep_chassis.drive_speed(x=control_vel_wheels[0], y=control_vel_wheels[1], z=0, timeout=5)
    use image detection
    if found:
        self.boundingbox = (0,0)
        confirm located

def approach():
    x_diff = 0
    y_size = 0
    matchheading()

def grab():
    gripping.GrabLego()


def passlego()
    setheadingtoline()
    approachline()
    communicate()

def recievelego()
    setheadingtoline()
    approachline()
    strafetobot()
    communicate()

def drop()
    searchfortarget()
    approachtarget()
    gripper release

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    finding_lego = True
    approaching_lego = False

    while True:
        if finding_lego:
            search()
        elif approaching_lego:
            approach():
        elif going_to_line:
            go_to_line():
        elif passing

