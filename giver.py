import numpy as np
from robomaster import robot
from robomaster import camera
import time
import math
import detection
import gripping

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
def move_to_lego(translation_speed = 0.20, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_camera=None):

    horizontal_distance = 1000000
    lego_dist = 100000
    goal_lego_dist = 40 # cm
    
    print('GIVER: Moving towards the Legos')

    while (np.abs(lego_dist - goal_lego_dist) > 5):
           
        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)

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

            if (lego_dist < 61): # WORKING ON THIS
                print('IM HERE')
                #ep_chassis.drive_speed(x=0, y=0, z=0)
                ep_chassis.drive_speed(x=, y=0, z=0)
                print('IM HERE AGAIN')
                #ep_chassis.move(x=0.21, y=0, z=0, xy_speed=0.3).wait_for_completed()
                print('IM HERE MOVE')
                ep_chassis.drive_speed(x=0, y=0, z=0)
                return

        else:
            ep_chassis.drive_speed(x=translation_speed, y=0,
                                z=0, timeout=5)
            
            time.sleep(0.1)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

        

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    
    ep_camera.start_video_stream(display=False)
    search_lego(ep_camera=ep_camera)
    move_to_lego(ep_camera=ep_camera)
    gripping.GrabLego(ep_gripper=ep_gripper, ep_arm=ep_arm)
