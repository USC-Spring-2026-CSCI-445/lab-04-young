#!/usr/bin/env python3
from math import inf
import queue
from time import sleep, time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# P controller class
class PController:
    """
    Generates control action taking into account instantaneous error (proportional action).
    """

    def __init__(self, kP, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        self.kP = kP
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = None

    def control(self, err, t):
        if self.t_prev is None:
            self.t_prev = t
            u = self.kP * err
            return max(self.u_min, min(self.u_max, u))

        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        self.t_prev = t
        u = self.kP * err
        u = max(self.u_min, min(self.u_max, u))
        return u


# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        self.kP = kP
        self.kD = kD
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = None
        self.err_prev = None

    def control(self, err, t):
        # On first call, skip derivative term
        if self.t_prev is None:
            self.t_prev = t
            self.err_prev = err
            u = self.kP * err
            return max(self.u_min, min(self.u_max, u))

        dt = t - self.t_prev
        if dt <= 1e-6:
            return 0

        d_err = (err - self.err_prev) / dt
        u = self.kP * err + self.kD * d_err
        u = max(self.u_min, min(self.u_max, u))

        self.t_prev = t
        self.err_prev = err
        return u


class RobotController:
    def __init__(self, desired_distance: float):
        print("\nMake the robot follow the wall on its left by maintaining the distance from it using LIDAR.\n")

        # ROS1 infrastructure
        rospy.init_node("robot_controller", anonymous=True)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Define controller for wall-following
        self.v0 = 0.15  # Base forward velocity (m/s)
        self.pd_controller = PController(kP=1.5, u_min=-1.0, u_max=1.0)

        self.desired_distance = desired_distance
        self.ir_distance = None

    def robot_laserscan_callback(self, lscan: LaserScan):
        left = lscan.ranges[80:100]
        left = [x for x in left if x != inf]
        if len(left) > 0:
            self.ir_distance = sum(left) / len(left)

    def control_loop(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.ir_distance is None:
                print("Waiting for IR sensor readings")
                sleep(0.1)
                continue

            ctrl_msg = Twist()

            # Compute error: positive = too far from wall
            err = self.ir_distance - self.desired_distance
            t = time()
            u = self.pd_controller.control(err, t)

            ctrl_msg.linear.x = self.v0
            ctrl_msg.angular.z = u

            self.robot_ctrl_pub.publish(ctrl_msg)
            print("ir: {:.3f}  err: {:.3f}  u: {:.3f}".format(self.ir_distance, err, u))
            rate.sleep()


if __name__ == "__main__":
    desired_distance = 0.5
    controller = RobotController(desired_distance)
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
