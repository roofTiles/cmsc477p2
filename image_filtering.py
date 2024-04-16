import cv2
import numpy as np
import math as mth
from matplotlib import pyplot as plt
from robomaster import robot
from robomaster import camera


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
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
                    yaw = mth.pi / 2 - thet

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

                    # orient robot
                    ep_chassis.speed(x=0, y=0, z=np.rad2deg(yaw))

                    # insert proportional controller here to move robot forward

                    # draw a line on the image
                    domain = [x[0], x[-1]]
                    rng = [y[0], y[-1]]
                    plt.plot(domain, rng, color="red", linewidth=1)
            except TypeError:
                print('Target line is out of view')

            plt.imshow(image)
            plt.show()
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            printpyt('Exiting')
            exit(1)