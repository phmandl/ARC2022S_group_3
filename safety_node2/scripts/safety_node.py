#!/usr/bin/env python3
from cmath import cos
import rospy
import numpy as np


# TODO: import ROS msg types and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String,Bool


class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.angle_min = 0
        self.angle_max = 0
        self.angle_incr=0
        self.range_min = 0
        self.range_max = 0

        # create ROS subscribers and publishers.
        rospy.Subscriber("/odom",Odometry,self.odom_callback)
        rospy.Subscriber("/scan",LaserScan,self.scan_callback)
        #init publishers
        self.pub_brake_bool = rospy.Publisher("brake_bool",Bool,queue_size=1)
        self.pub_brake = rospy.Publisher("brake",AckermannDriveStamped,queue_size=1)



    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed=odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC

        if(self.speed!=0):
            print( self.speed)
            angle_current=self.angle_min

            for distance in scan_msg.ranges:
                range_rate=cos(angle_current)
            rospy.loginfo("Front Distance: %f", scan_msg.ranges[540]) 


        # TODO: publish brake message and publish controller bool
        pass


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    print("bla ala bla")
    rospy.spin()
if __name__ == '__main__':
    main()