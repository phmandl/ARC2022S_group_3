#!/usr/bin/env python3
from __future__ import print_function
from ast import If
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from dynamic_reconfigure.server import Server
from wall_follow.cfg import dyn_reconfigConfig

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
NR_SCANS = 1080
THETA_ANGLE = 60
ANGLE_INCREMENT = 0.004365153145045042
INDEX_RIGHT_SCAN_RAY = 180
INDEX_LEFT_SCAN_RAY = 900
LOOKAHEAD_DISTANCE = 0.5
DESIRED_DISTANCE_RIGHT = 1.0 # meters
DESIRED_DISTANCE_LEFT = 1.0
MAX_VELOCITY = 1.50 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
HZ = 100
TS = 1.0/HZ
NR_RAYS_A_TO_B = int(np.deg2rad(THETA_ANGLE)/ANGLE_INCREMENT)

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        # self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)      
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)

        # dynamic reconfigure server
        self.srv = Server(dyn_reconfigConfig, self.dynCallback)

        # rosrate
        self.r = rospy.Rate(HZ)

        # PID Stuff
        self.Kp = 1
        self.Ki = 0.1
        self.Kd = 0.0
        self.N = 100
        self.e0 = 0
        self.e1 = 0
        self.e2 = 0
        self.u0 = 0
        self.u1 = 0
        self.u2 = 0
        self.Dt = 0
        self.Dtp1 = 0
        self.alpha = 0
        self.angle = 0
        self.maxVelo = MAX_VELOCITY

    def pid_control(self):

        # Discrete Time PID see https://www.scilab.org/discrete-time-pid-controller-implementation
        a0 = (1 + self.N*TS)
        a1 = -1*(2 + self.N*TS)
        a2 = 1
        b0 = self.Kp*(1 + self.N*TS) + self.Ki*TS*(1 + self.N*TS) + self.Kd*self.N
        b1 = -1*(self.Kp*(2 + self.N*TS) + self.Ki*TS + 2*self.Kd*self.N)
        b2 = self.Kp + self.Kd*self.N
        ku1 = a1/a0; ku2 = a2/a0; ke0 = b0/a0; ke1 = b1/a0; ke2 = b2/a0

        self.u0 = -ku1*self.u1 - ku2*self.u2 + ke0*self.e0 + ke1*self.e1 + ke2*self.e2

        # invert angle argument
        self.angle = -1*self.u0

        # update values
        self.e1 = self.e0
        self.e2 = self.e1
        self.u1 = self.u0
        self.u2 = self.u1

    def followLeft(self):

        # CHECK VELOCITY ON LAST STEERING ANGLE
        abs_deg_angle = np.abs(np.rad2deg(self.angle))
        if (abs_deg_angle <= 10):
            # velocity = 1.5
            velocity = self.maxVelo
        elif (abs_deg_angle <= 20):
            velocity = 1.0
        else:
            velocity = 0.5

        # Ackermann Msgs
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.angle
        drive_msg.drive.speed = self.maxVelo
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """ 
        """
        # Get distances a and b to the wall
        b_range = data.ranges[INDEX_LEFT_SCAN_RAY]
        a_range = data.ranges[INDEX_LEFT_SCAN_RAY - NR_RAYS_A_TO_B]

        # Recalculate theta since it is not accurate
        theta = ANGLE_INCREMENT*NR_RAYS_A_TO_B

        # calc angle
        self.alpha = np.arctan2(a_range*np.cos(theta) - b_range, a_range*np.sin(theta))
 
        # Calc Dt
        self.Dt = b_range*np.cos(self.alpha)

        # Estimate future distance
        self.Dtp1 = self.Dt + LOOKAHEAD_DISTANCE*np.sin(self.alpha)

        # calculate error
        self.e0 = DESIRED_DISTANCE_LEFT - self.Dtp1

    def dynCallback(self, config, level):
        self.Kp = config.Kp
        self.Ki = config.Ki
        self.Kd = config.Kd
        self.N = config.N
        self.maxVelo = config.maxVelo
        return config

def main(args):
    rospy.init_node("wall_follow", anonymous=True)
    wf = WallFollow()

    while not rospy.is_shutdown():
        wf.pid_control()
        wf.followLeft()
        wf.r.sleep()

if __name__=='__main__':
	main(sys.argv)

