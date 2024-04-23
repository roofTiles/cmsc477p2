import numpy as np
from robomaster import robot
from robomaster import camera
import time
import math
import detection
import gripping
#import messagingserver

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)

# rotates until the heading of the lego
# is towards the robot

def search_lego(rotational_speed = 20, k = 0.01, ep_camera=None): 
    
    distance = 1000000
    print('GIVER: Seaching for Legos')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 50: # bounding box x-center must be 200 away from center of img

        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            horizontal_center = bb[0] + bb[2]/2
            distance = horizontal_center - 640/2 # finding error in horizontal
            ep_chassis.drive_speed(x=0, y=0, z=k * distance * rotational_speed, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
    print('GIVER: Facing the Legos')

# tell robot to move towards the lego tower
# k_t is translational proportional controller
# k_r is rotational proportional controller
# WORK IN PROGRESS
def move_to_lego(translation_speed = 0.05, rotational_speed = 10, ep_camera=None):

    results = detection.detect_object_in_image('lego', ep_camera=ep_camera)
    bb = results[1]
    print(bb)

    while bb[2] < 76:
        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)
        bb = results[1]
        print(bb)
        ep_chassis.drive_speed(x=0.05, y=0, z=0, timeout=5)
        time.sleep(0.1)
    
    ep_chassis.drive_speed(x=0.00, y=0, z=0, timeout=5)

        

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    search_lego(ep_camera=ep_camera)
    move_to_lego(ep_camera=ep_camera)


    ep_gripper.open(power=100)     #open gripper   
    ep_camera.start_video_stream(display=True)


