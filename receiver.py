import numpy as np
from robomaster import robot
from robomaster import camera
import time
import math
import detection
import gripping

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)


# tell robot to move towards the giver
# k_t is translational proportional controller
# k_r is rotational proportional controller
def move_to_robot(translation_speed = 0.40, rotational_speed = 40, 
                 k_t = 0.01/2, k_r = 0.05, ep_camera=None):

    horizontal_distance = 1000000
    lego_dist = 100000
    goal_robot_dist = 30 # cm
    
    print('RECEIVER: Moving towards giver')

    while (np.abs(lego_dist - goal_robot_dist) > 5):
           
        results = detection.detect_object_in_image('robot', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            robot_dist = 1/(math.tan((bb[3]/384 * 68 * math.pi)/180.0)) * 37 # gives distance to robot in cm
            horizontal_center = bb[0] + bb[2]/2
            distance_error = goal_robot_dist - robot_dist # finding error in vertical
            horizontal_distance = horizontal_center - 320 # finding error in horizontal

            print(robot_dist)

            if (horizontal_distance > 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)

            if (horizontal_distance <= 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z=0, timeout=5)

            if (robot_dist < 41):
                print("RECEIVER: MOVING TOWARDS GIVER, NOT USING CAMERA ANYMORE")
                speed = 0.075
                ep_chassis.drive_speed(x=speed, y=0, z=0) # drive towards robot
                time.sleep(0.35/speed)
                ep_chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.1)
                return

        else:
            ep_chassis.drive_speed(x=translation_speed, y=0,
                                z=0, timeout=5)
            
            time.sleep(0.1)
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)


# make reeciver face endpoint
def search_endpoint(rotational_speed = 10, k = 0.01, ep_camera=None): 
    
    distance = 1000000
    print('RECEIVER: Seaching for Endpoint')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 20: # bounding box x-center must be 15 away from center of img

        results = detection.detect_endpoint(ep_camera=ep_camera, show=False)

        if results[0]: # if lego in FOV
            bb = results[1] # bounding box -  array of format [x,y,w,h]
            horizontal_center = bb[0] + bb[2]/2
            distance = horizontal_center - 1280/2 # finding error in horizontal
            ep_chassis.drive_speed(x=0, y=0, z=k * distance * rotational_speed, timeout=5)

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop rotating
    print('RECEIVER: Facing Endpoint')

# make reeciver go to endpoint
def move_to_endpoint(translation_speed = 0.04, rotational_speed = 10, 
                 k_t = 0.01, k_r = 0.05, ep_camera=None): 
    
    height = 0
    print('RECEIVER: Moving to Endpoint')

    goal_height = 460

    prev_height = 0
    count = 0

    # want bounding box as close to center of img in horizontal dir
    while height < goal_height:
        
        results = detection.detect_endpoint(ep_camera=ep_camera, show=False)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            height = bb[1]
            height_error = (goal_height+20) - height # finding height in error

            horizontal_center = bb[0] + bb[2]/2
            horizontal_distance = horizontal_center - 1280/2 # finding error in horizontal


            if (horizontal_distance > 100):
                ep_chassis.drive_speed(x=translation_speed * k_t * height_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)

            if (horizontal_distance <= 100):
                ep_chassis.drive_speed(x=translation_speed * k_t * height_error, y=0,
                                z=0, timeout=5)

            if prev_height == height:
                count = count + 1
            if prev_height != height:
                count = 0
            prev_height = height

            if count == 3:
                break

        else:
            ep_chassis.drive_speed(x=0, y=0, z=rotational_speed, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop moving
    time.sleep(0.1)
    print('RECEIVER: At Endpoint')



if __name__ == '__main__':
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    
    ep_camera.start_video_stream(display=False)
    gripping.LookDown(ep_gripper=ep_gripper, ep_arm=ep_arm, x=0, y=50) # have arm down before looking for endpoint
    search_endpoint(ep_camera=ep_camera)
    move_to_endpoint(ep_camera=ep_camera)
    
