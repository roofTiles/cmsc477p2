import cv2
import time
import numpy as np
import math as mth
from matplotlib import pyplot as plt
from robomaster import robot
from robomaster import camera


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
<<<<<<< HEAD
    ep_chassis = ep_robot.chassis
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    
    Oriented = False
=======
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
>>>>>>> aae9c7c8aa62d461efc73e4c19b516efd6e56243

    while not Oriented:
        try:
            # Input camera feed
            image = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            # image = cv2.imread('C:/Users/flori/OneDrive/CMSC477/Project_02/image0.png')

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

<<<<<<< HEAD
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
=======
                    # orient robot
                    ep_chassis.move(x=0, y=0, z=np.rad2deg(yaw), z_speed=20).wait_for_completed()
>>>>>>> aae9c7c8aa62d461efc73e4c19b516efd6e56243


            except TypeError:
                print('Target line is out of view')
                # orient robot
<<<<<<< HEAD
                ep_chassis.move(x=0, y=0, z=-15, z_speed=30).wait_for_completed()
=======
                ep_chassis.move(x=0, y=0, z=np.rad2deg(np.pi/6), z_speed=20).wait_for_completed()
>>>>>>> aae9c7c8aa62d461efc73e4c19b516efd6e56243

            # plt.imshow(image)
            # plt.show()
            cv2.imshow("img", image)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
<<<<<<< HEAD
            print('Exiting')
=======
            printpyt('Exiting')
>>>>>>> aae9c7c8aa62d461efc73e4c19b516efd6e56243
            exit(1)
