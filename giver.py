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
def move_to_lego(translation_speed = 0.20, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_camera=None):

    horizontal_distance = 1000000
    lego_dist = 100000
    goal_lego_dist = 30 # cm
    looking_down = False
    looking_down_2 = False

    print('GIVER: Moving towards the Legos')

    while (np.abs(lego_dist - goal_lego_dist) > 5):
           
        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            lego_dist = 1/(math.tan((bb[2]/640 * 120 * math.pi)/180.0)) * 10 # gives distance to lego in cm
            horizontal_center = bb[0] + bb[2]/2
            print("Width: " + str(bb[2]))
            print("Top Height: " + str(bb[1]))
            distance_error = 0 - lego_dist # finding error in vertical
            horizontal_distance = horizontal_center - 320 - 20 # finding error in horizontal

            print(lego_dist)

            if (horizontal_distance > 5):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)
                print("ROTATING: " + str(rotational_speed * k_r * horizontal_distance))

            if (horizontal_distance <= 5):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z=0, timeout=5)
                
            if (lego_dist < 60 or bb[1] > 210) and not looking_down:
                gripping.LookDown(ep_arm=ep_arm)
                looking_down = True

            elif (lego_dist < 45 or bb[1] > 210) and not looking_down_2:
                gripping.LookDown(ep_arm=ep_arm)
                looking_down_2 = True

            elif (lego_dist < 40 or (bb[1] > 210 and looking_down_2)):
                print("GIVER: MOVING TOWARDS LEGO TOWER, NOT USING CAMERA ANYMORE")
                speed = 0.065
                ep_chassis.drive_speed(x=speed, y=0, z=0) # drive towards lego
                time.sleep(.2/speed)
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.1)
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
    
    ep_camera.start_video_stream(display=True)
    search_lego(ep_camera=ep_camera)
    move_to_lego(ep_camera=ep_camera)
    print("GRABBING")
    gripping.GrabLego(ep_gripper=ep_gripper, ep_arm=ep_arm)
    time.sleep(2)
    gripping.DropLego(ep_gripper=ep_gripper, ep_arm=ep_arm)
