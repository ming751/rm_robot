from Robotic_Arm.rm_robot_interface import *
import time
import threading

def connect_robot(ip, port, level=3, mode=None):
    """
    Connect to the robot arm.

    Args:
        ip (str): IP address of the robot arm.
        port (int): Port number.
        level (int, optional): Connection level. Defaults to 3.
        mode (int, optional): Thread mode as an integer (0: single, 1: dual, 2: triple). Defaults to None.

    Returns:
        RoboticArm: Instance of the connected RoboticArm.
    """
    if mode is not None:
        thread_mode = rm_thread_mode_e(mode)
        robot = RoboticArm(thread_mode)
    else:
        robot = RoboticArm()

    handle = robot.rm_create_robot_arm(ip, port, level)

    if handle.id == -1:
        print("\nFailed to connect to the robot arm\n")
        exit(1)
    else:
        print(f"\nSuccessfully connected to the robot arm: {handle.id}\n")

    return robot,handle

def disconnect_robot(robot):
    """
    Disconnect from the robot arm.

    Args:
        robot (RoboticArm): Instance of the RoboticArm.

    Returns:
        None
    """
    handle = robot.rm_delete_robot_arm()
    if handle == 0:
        print("\nSuccessfully disconnected from the robot arm\n")
    else:
        print("\nFailed to disconnect from the robot arm\n")

def demo_movej(robot, joint=None, v=10, r=0, connect=0, block=0):
    """
    Perform movej motion.

    Args:
        robot (RoboticArm): Instance of the RoboticArm.
        joint (list of float, optional): Joint positions. Defaults to [0, 0, 0, 0, 0, 0].
        v (float, optional): Speed of the motion. Defaults to 20.
        connect (int, optional): Trajectory connection flag. Defaults to 0.
        block (int, optional): Whether the function is blocking (1 for blocking, 0 for non-blocking). Defaults to 1.
        r (float, optional): Blending radius. Defaults to 0.

    Returns:
        None
    """
    if joint is None:
        joint = [0, 0, 0, 0, 0, 0]
    movej_result = robot.rm_movej(joint, v, r, connect, block)
    if movej_result == 0:
        print("\nmovej motion succeeded\n")
    else:
        print("\nmovej motion failed, Error code: ", movej_result, "\n")

def main():
    robot,robot_handle = connect_robot("192.168.1.18", 8080, 3, 2)



    start_time = time.time()

    for i in range(10):

        demo_movej(robot, [-90, -60, 120, 00, -60, 2*i], 15, 0, 0, 0)
        print(robot.rm_get_current_arm_state())

    disconnect_robot(robot)
    print("Time taken: ", time.time() - start_time)


if __name__ == "__main__":
    main()