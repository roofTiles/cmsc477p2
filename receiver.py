import numpy as np
from robomaster import robot
from robomaster import camera
import time
import math
import detection
import gripping
import messagingclient

# Defines functionality that is specific
# to the robot handing off the lego tower (the giver)

# have receiver strafe to the giver
def strafe_to_giver(translational_speed = 0.075, k = 0.05, ep_camera=None):
    
    distance = 1000000
    print('RECEIVER: Strafing to Giver robot')

    lastTime = time.time() # get time started search
    
    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 10: # bounding box x-center must be 10 away from center of img

        results = detection.detect_object_in_image(c='robot', ep_camera=ep_camera)

        if results[0]: # if lego in FOV
            bb = results[1] # bounding box -  array of format [x,y,w,h]
            horizontal_center = bb[0]
            distance = horizontal_center - 320 # finding error in horizontal

            control = translational_speed * distance * k
            
            print("control: ", translational_speed * distance * k)

            if np.abs(control) < 0.06: # around when actuator starts acting weird
                break
            
            ep_chassis.drive_speed(x=0, y=translational_speed * distance * k, z=0, timeout=5)

        else:

            currTime = time.time()
            if currTime-lastTime > 4: # switch directions if haven't seen robot in 2s
                translational_speed = translational_speed*-1
            
            ep_chassis.drive_speed(x=0, y=translational_speed, z=0, timeout=5)

        time.sleep(0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5) # stop moving
    time.sleep(0.1)
    print('RECEIVER: Facing Giver')

# have receiver move to giver
def move_to_giver(translation_speed = 0.2, rotational_speed = 10, 
                 k_t = 0.005, k_r = 0.1, ep_camera=None):
    
    horizontal_distance = 1000000
    giver_y = 100000
    goal_giver_y = 255 # px

    print('RECEIVER: Moving towards the robot')

    while (np.abs(giver_y - goal_giver_y) > 5):
           
        results = detection.detect_object_in_image('robot', ep_camera=ep_camera, conf=0.6)

        if results[0]: # if robot in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            giver_y = bb[1]
            horizontal_center = bb[0]
            print("Y: " + str(bb[1]))
            distance_error = giver_y - goal_giver_y
            horizontal_distance = horizontal_center - 320 # finding error in horizontal

            if (horizontal_distance > 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)

            if (horizontal_distance <= 10):
                ep_chassis.drive_speed(x=-1*translation_speed * k_t * distance_error, y=0,
                                z=0, timeout=5)
 
            elif (giver_y > goal_giver_y):
                break           

        else:
            ep_chassis.drive_speed(x=translation_speed, y=0,
                                z=0, timeout=5)
            
            time.sleep(0.1)
            
    ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
    time.sleep(0.1)

    print("RECEIVER: At Giver")
    return


# make reeciver face endpoint
def search_endpoint(rotational_speed = 10, k = 0.01, ep_camera=None): 
    
    distance = 1000000
    print('RECEIVER: Seaching for Endpoint')

    # want bounding box as close to center of img in horizontal dir
    while np.abs(distance) > 20: # bounding box x-center must be 15 away from center of img

        results = detection.detect_endpoint(ep_camera=ep_camera, show=False)

        if results[0]: # if lego in FOV
            bb = results[1] # bounding box -  array of format [x,y,w,h]
            horizontal_center = bb[0]
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

            horizontal_center = bb[0]
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
    
    ep_camera.start_video_stream(display=True)

    # orient with line

    # wait for giver to send message that ready to pass

    # for strafing to giver
    strafe_to_giver(ep_camera=ep_camera)

    # moving to giver
    gripping.LookDown(ep_arm=ep_arm, x = 80, y = 70)
    move_to_giver(ep_camera=ep_camera)
    ep_gripper.close(power=100)

    # send message to giver saying grabbed lego
    messagingclient.SendGrabMessage(0)
    
    # for going to endpoint
    
    #gripping.LookDown(ep_gripper=ep_gripper, ep_arm=ep_arm, x=0, y=50) # have arm down before looking for endpoint
    #search_endpoint(ep_camera=ep_camera)
    #move_to_endpoint(ep_camera=ep_camera)
    
