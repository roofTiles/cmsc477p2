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
def search_lego(rotational_speed = 20, k = 0.005, ep_camera=None): 
    
    distance = 1000000
    print('GIVER: Seaching for Legos')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 10: # bounding box x-center must be 10 away from center of img

        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            horizontal_center = bb[0] + bb[2]/2
            distance = horizontal_center - 320 # finding error in horizontal
            ep_chassis.drive_speed(x=0, y=0, z=k * distance * rotational_speed, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
    print('GIVER: Facing the Legos')

# tell robot to move towards the lego tower
# WORK IN PROGRESS
def move_to_lego(rotational_speed = 20, translation_speed = 0.25, k = 0.01, ep_camera=None):

    vertical_distance = horizontal_distance = 1000000
    print('GIVER: Moving towards the Legos')

    while (np.abs(vertical_distance) > 10):
        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            vertical_center = bb[1] + bb[3]/2
            horizontal_center = bb[0] + bb[2]/2
            vertical_distance = vertical_center - 2000 # finding error in vertical
            horizontal_distance = horizontal_center - 320 # finding error in horizontal

            if (horizontal_distance > 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k * vertical_distance, y=0,
                                z= rotational_speed * k * horizontal_distance, timeout=5)

            if (horizontal_distance <= 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k * vertical_distance, y=0,
                                z=0, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)

        time.sleep(0.1)

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
