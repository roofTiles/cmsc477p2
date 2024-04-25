import queue
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from robomaster import robot

import cv2
import time
from robomaster import camera
import csv
import detection
import gripping
import messagingclient


ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_chassis = ep_robot.chassis

def StrafeToPartner(ep_camera=None):
    seeing_partner = False
    tspeed = 0.2
    while not seeing_partner:
        lastTime = time.time()
        if search_robot(translation_speed=tspeed, k=.01, ep_camera=ep_camera) < 50:
            not_seeing_partner = True
        elif time.time()-lastTime > 6:
            tspeed = -tspeed
            


def search_robot(translation_speed = 20, k = 0.01, ep_camera=None): 
    
    distance = 1000000
    print('RECEIVER: Seaching for Legos')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 50: # bounding box x-center must be 200 away from center of img

        results = detection.detect_object_in_image('robot', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            horizontal_center = bb[0] + bb[2]/2
            distance = horizontal_center - 640/2 # finding error in horizontal
            ep_chassis.drive_speed(x=0, z=k * distance * translation_speed, z=0, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=translation_speed, z=0, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
    print('RECEIVER: Facing the Legos')
    return distance

def move_to_robot(translation_speed = 0.20, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_camera=None):

    horizontal_distance = 1000000
    lego_dist = 100000
    goal_lego_dist = 40 # cm
    
    print('RECEIVER: Moving towards the Legos')

    while (np.abs(lego_dist - goal_lego_dist) > 5):
           
        results = detection.detect_object_in_image('robot', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            lego_dist = 1/(math.tan((bb[3]/384 * 68 * math.pi)/180.0)) * 16 # gives distance to lego in cm
            horizontal_center = bb[0] + bb[2]/2
            distance_error = goal_lego_dist - lego_dist # finding error in vertical
            horizontal_distance = horizontal_center - 320 # finding error in horizontal

            print(lego_dist)

            if (horizontal_distance > 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)

            if (horizontal_distance <= 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z=0, timeout=5)

            if (lego_dist < 61):
                print("RECEIVER: MOVING TOWARDS LEGO TOWER, NOT USING CAMERA ANYMORE")
                speed = 0.075
                ep_chassis.drive_speed(x=speed, y=0, z=0) # drive towards lego
                time.sleep(0.60/speed)
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.1)
                return

        else:
            ep_chassis.drive_speed(x=translation_speed, y=0,
                                z=0, timeout=5)
            
            time.sleep(0.1)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

def goTarget(translation_speed = 0.20, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_camera = None):
        results = detection.detect_object_in_image('robot', ep_camera=ep_camera)

        




if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis

    y_val = 0.2

    StrafeToPartner()

    ep_robot.close()

