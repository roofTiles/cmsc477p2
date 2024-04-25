import numpy as np
from robomaster import robot
from robomaster import camera
import time
import math
import detection
import gripping
import messagingserver

# tell robot to move towards the lego tower
# k_t is translational proportional controller
# k_r is rotational proportional controller
# WORK IN PROGRESS
def move_to_lego(translation_speed = 0.065, rotational_speed = 10, 
                 k_t = 0.01/2, k_r = 0.01, ep_camera=None):

    results = detection.detect_object_in_image('lego', ep_camera=ep_camera)
    bb = results[1]

    while bb[2] < 85:
        #strafe left or right
        while abs(bb[0] - 320) > 5:
            results = detection.detect_object_in_image('lego', ep_camera=ep_camera)
            bb = results[1]
            if bb[0] > 320:
                ep_chassis.drive_speed(x=0, y=translation_speed, z=0, timeout=5)
            elif bb[0] < 320:
                ep_chassis.drive_speed(x=0, y=-translation_speed, z=0, timeout=5)
            print('x: ' , bb[0] , ' ' , 'y: ', bb[1] , 'w: ' , bb[2])
            time.sleep(0.1)
        
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        results = detection.detect_object_in_image('lego', ep_camera=ep_camera)
        bb = results[1]
        #print('Width: ' , bb[2])
        print('x: ' , bb[0] , ' ' , 'y: ', bb[1] , 'w: ' , bb[2])
        ep_chassis.drive_speed(x=translation_speed, y=0, z=0, timeout=5)
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
    #search_lego(ep_camera=ep_camera)
    ep_gripper.open(power=100)
    move_to_lego(ep_camera=ep_camera)
    ep_gripper.close(power=100)
    #gripping.GrabLego(ep_gripper=ep_gripper, ep_arm=ep_arm)

    # move to line
    # send message to receiver that ready for pass

    # wait for receiver to grab
    #messagingserver.StartPassingComms(ep_gripper)
