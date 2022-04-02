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

#PID CONTROL PARAMS
kp = 14#TODO
kd = 1#TODO
ki = 0#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
NR_SCANS = 1080
THETA_ANGLE = 20
ANGLE_INCREMENT = 0.004365153145045042
INDEX_B_SCAN = int(np.deg2rad(ANGLE_RANGE - 90)/ANGLE_INCREMENT)
INDEX_A_SCAN = int(np.deg2rad(ANGLE_RANGE - 90 - THETA_ANGLE)/ANGLE_INCREMENT)
LOOKAHEAD_DISTANCE = 1
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.9
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        # self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)      
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        return 0.0

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd

        angle = -1*kp*error #- (error - prev_error)*kd

        prev_error = error

        # CHECK VELOCITY ON LAST STEERING ANGLE
        # if (0 <= np.abs(np.rad2deg(angle)) <= 10):
        #     velocity = 1.5
        # elif (10 < np.abs(np.rad2deg(angle)) <= 20):
        #     velocity = 1.0
        # else:
        #     velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 

        # Get distances a and b to the wall
        a = data.ranges[INDEX_A_SCAN + 1]
        b = data.ranges[INDEX_B_SCAN + 1]

        # calc angle
        alpha = np.arctan2(a*np.cos(np.deg2rad(THETA_ANGLE))-b,a*np.sin(np.deg2rad(THETA_ANGLE)))

        # Calc Dt
        Dt = b*np.cos(alpha)

        print(b)

        # Estimate future distance
        Dtp1 = Dt + LOOKAHEAD_DISTANCE*np.sin(alpha)

        # calculate error
        error = leftDist - Dtp1
        return error

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)

        # Send to PID
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)