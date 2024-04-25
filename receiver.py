import numpy as np
from robomaster import robot
from robomaster import camera
import time
import math
import detection
import gripping
import messagingclient

# orienting to blue line
# WORK IN PROGRESS
def blue_line_orient(ep_camera=None):
    Oriented = False
    while not Oriented:
        try:
            # Input camera feed
            image = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)

            # apply gaussian blur
            Gaussian = cv2.GaussianBlur(image, (13, 9), 0)

            hsv = cv2.cvtColor(Gaussian, cv2.COLOR_BGR2HSV)

            # Threshold of blue in HSV space
            lower_blue = np.array([60, 60, 130])  # [60, 35, 140]
            upper_blue = np.array([160, 220, 255])  # [180, 255, 255]

            # preparing the mask to overlay
            mask2 = cv2.inRange(hsv, lower_blue, upper_blue)

            # The black region in the mask has the value of 0,
            # so when multiplied with original image removes all non-blue regions
            result = cv2.bitwise_and(Gaussian, Gaussian, mask=mask2)

            # Apply edge detection method on the image
            edges = cv2.Canny(mask2, 50, 150, apertureSize=3)

            # This returns an array of r and theta values
            lines = cv2.HoughLines(edges, 1, np.pi / 180, 200)

            r_vals = 0
            thet_vals = 0
            try:

                # average all lines
                for i in range(len(lines)):
                    r_vals = r_vals + lines[i][0][0]
                    thet_vals = thet_vals + lines[i][0][1]
                    r = r_vals / (i + 1)
                    thet = thet_vals / (i + 1)

                    # yaw angle
                    yaw = (mth.pi / 2) - thet 
                    print("yaw", np.rad2deg(yaw))
                    
                    y = []
                    x = []

                # determine equation of mean line to plot
                if thet < mth.pi / 2:
                    x0 = r / mth.cos(thet)
                    y0 = r / mth.cos(mth.pi / 2 - thet)
                    m = -y0 / x0

                elif thet > mth.pi / 2:
                    x0 = -r / mth.cos(mth.pi - thet)
                    y0 = r / mth.cos(thet - mth.pi / 2)
                    m = -y0 / x0

                for j in range(0, 641):
                    if m * j + y0 < 360:
                        x.append(j)
                        y.append(m * j + y0)

                # draw a line on the image
                # domain = [x[0], x[-1]]
                # rng = [y[0], y[-1]]
                # plt.plot(domain, rng, color="red", linewidth=1)
                start_pt = (round(x[0]), round(y[0]))
                end_pt = (round(x[-1]), round(y[-1]))
                print("x0 = " + str(x[0]))
                print("y0 = " + str(y[0]))
                print("x1 = " + str(x[-1]))
                print("y1 = " + str(y[-1]))
                
                cv2.line(image, start_pt, end_pt, (255, 0, 0), thickness = 2)

                # orient robot
                ep_chassis.move(x=0, y=0, z=np.rad2deg(yaw), z_speed=0.3*np.rad2deg(yaw)).wait_for_completed()
                
                # ep_chassis.drive_speed(x=0, y=0, z=-np.rad2deg(yaw), timeout=5)
                time.sleep(1)

                if np.rad2deg(yaw) > -3 and np.rad2deg(yaw) < 3:
                    Oriented = True
                    # move robot
                    y_min = 110  # pixels 124
                    y_end = 270  # pixels 285
                    ws_inside_dist = 1.15  # [m]
                    scale = ws_inside_dist/(y_end - y_min)
                    pixel_dist = round((y[0] + y[-1])/2)
                    x_vel = abs(pixel_dist-y_end)*scale
                    print("pixel distance:", pixel_dist)
                    print("actual distance", x_vel)
                    # ep_chassis.drive_speed(x=x_vel, y=0, z=0, timeout=5)
                    # time.sleep(3)
                    ep_chassis.move(x=x_vel, y=0, z=0, z_speed=0).wait_for_completed()

                    # orient robot
                    ep_chassis.move(x=0, y=0, z=np.rad2deg(yaw), z_speed=20).wait_for_completed()

            except TypeError:
                print('Target line is out of view')
                
                # orient robot
                ep_chassis.move(x=0, y=0, z=-15, z_speed=30).wait_for_completed()
                ep_chassis.move(x=0, y=0, z=np.rad2deg(np.pi/6), z_speed=20).wait_for_completed()

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
    
    width = 0
    print('RECEIVER: Moving to Endpoint')

    goal_width = 800

    prev_width = 0
    count = 0

    # want bounding box as close to center of img in horizontal dir
    while width < goal_width:

        print(width)
        
        results = detection.detect_endpoint(ep_camera=ep_camera, show=False)

        if results[0]: # if lego in FOV
            
            bb = results[1] # bounding box -  array of format [x,y,w,h] scaled to image size of (384, 640)
            width = bb[2]
            width_error = (goal_width) - width # finding height in box width

            horizontal_center = bb[0]
            horizontal_distance = horizontal_center - 1280/2 # finding error in horizontal


            if (horizontal_distance > 100):
                ep_chassis.drive_speed(x=translation_speed * k_t * width_error, y=0,
                                z= rotational_speed * k_r * horizontal_distance, timeout=5)

            if (horizontal_distance <= 100):
                ep_chassis.drive_speed(x=translation_speed * k_t * width_error, y=0,
                                z=0, timeout=5)

            if prev_width >= width:
                count = count + 1
            else:
                count = 0
            prev_width = width

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
    blue_line_orient(ep_camera=ep_camera)

    # wait for giver to send message that ready to pass

    # for strafing to giver
    #strafe_to_giver(ep_camera=ep_camera)

    # moving to giver
    '''
    gripping.LookDown(ep_arm=ep_arm, x = 80, y = 70)
    move_to_giver(ep_camera=ep_camera)
    ep_gripper.close(power=100)
    '''

    # send message to giver saying grabbed lego
    #messagingclient.SendGrabMessage(0)
    
    # for going to endpoint
    '''
    gripping.LookUp(ep_arm=ep_arm, x = 80, y = 70)
    gripping.LookDown(ep_gripper=ep_gripper, ep_arm=ep_arm, x=0, y=50) # have arm down before looking for endpoint
    search_endpoint(ep_camera=ep_camera)
    move_to_endpoint(ep_camera=ep_camera)
    ep_gripper.open(power=100)
    '''
    
