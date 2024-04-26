from robomaster import robot
import time

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper


    x_to_giver = 0.5
    y_to_giver = 0.5
    z_to_giver = 90

    x_to_target1 = -0.5
    x_to_target2 = 0.5
    z_to_target = 90


    # rotate receiver towards giver
    ep_chassis.move(x=0, y=0, z=z_to_giver, z_speed=45).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # move gripper into place and open
    ep_arm.move(x=1000, y=-1000).wait_for_completed()
    ep_gripper.open(power=50)

    # wait 1s
    time.sleep(1)

    # move receiver to giver
    ep_chassis.move(x=0, y=y_to_giver, z=0, xy_speed=0.5).wait_for_completed()
    ep_chassis.move(x=x_to_giver, y=0, z=0, xy_speed=0.5).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # close gripper
    ep_gripper.close(power=50)

    # wait 1s
    time.sleep(1)

    # back receiver away from giver
    ep_chassis.move(x=x_to_target1, y=0, z=0, xy_speed=0.5).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # rotate receiver to target
    ep_chassis.move(x=0, y=0, z=z_to_target, z_speed=45).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # move receiver to target
    ep_chassis.move(x=x_to_target2, y=0, z=0, xy_speed=0.5).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # move and open gripper to set lego block on target
    ep_arm.move(x=1000, y=-1000).wait_for_completed()
    ep_gripper.open(power=50)

    ep_robot.close()