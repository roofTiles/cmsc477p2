from robomaster import robot
import time

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper


    x_to_block = 0.5
    z_to_block = 180

    z_to_line = 90
    x_to_line = 0.5

    # rotate giver 180 degrees to block
    ep_chassis.move(x=0, y=0, z=z_to_block, z_speed=45).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # move gripper into place and open
    ep_arm.move(x=1000, y=-1000).wait_for_completed()
    ep_gripper.open(power=50)

    # wait 1s
    time.sleep(1)

    # move giver to block 
    ep_chassis.move(x=x_to_block, y=0, z=0, xy_speed=0.5).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # close gripper
    ep_gripper.close(power=50)

    # wait 1s
    time.sleep(1)

    # rotate giver 90 degrees to block
    ep_chassis.move(x=0, y=0, z=z_to_line, z_speed=45).wait_for_completed()

    # wait 1s
    time.sleep(1)

    # move giver to line 
    ep_chassis.move(x=x_to_line, y=0, z=0, xy_speed=0.5).wait_for_completed()

    # wait 10s for reciever bot
    time.sleep(10)

    # open gripper
    ep_gripper.open(power=50)

    ep_robot.close()