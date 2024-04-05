import numpy as np
from robomaster import robot
from robomaster import camera
import time
import detection
import gripping

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)

# rotates until the heading of the lego
# is towards the robot

def search_lego(rotational_speed = 30, k = 0.01, ep_camera=None): 
    
    distance = 1000000
    print('GIVER: Seaching for Legos')

    # want bounding box as close to center of img in horizontal dir
    while distance > 30:

        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            bb = results[1] # bounding box -  array of format [x,y,w,h]
            horizontal_center = bb[0] + bb[2]/2

            distance = 320 - horizontal_center

            ep_chassis.drive_speed(x=0, y=0, z=k * distance * rotational_speed,
                                   timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        print(distance)

        time.sleep(0.1)


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    search_lego(ep_camera=ep_camera)
