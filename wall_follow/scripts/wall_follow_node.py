#!/usr/bin/env python
# from __future__ import print_function
# from ast import If
import sys
# import math
# from urllib.parse import MAX_CACHE_SIZE
import numpy as np

#ROS Imports
import rospy
# from std_msgs.msg import Float64
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
DESIRED_DISTANCE_RIGHT = 1.0 # meters
DESIRED_DISTANCE_LEFT = 1.3
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
HZ = 100
TS = 1.0/HZ
NR_RAYS_A_TO_B = int(np.deg2rad(THETA_ANGLE)/ANGLE_INCREMENT)
MAX_VELOCITY = 4.5
MAX_LOOKAHEAD = 1.0

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        Dtp1_topic = '/dtp1'

        # self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)      
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)
        #self.Dtp1_pub = rospy.Publisher(Dtp1_topic, Float64, queue_size=100)

        # dynamic reconfigure server
        self.srv = Server(dyn_reconfigConfig, self.dynCallback)

        # rosrate
        self.r = rospy.Rate(HZ)

        # PID Stuff
        self.Kp = 0.8
        self.Ki = 0.0
        self.Kd = 0.001
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
        self.velocity = 0
        self.midRayRange = 0
        self.lookahead = MAX_LOOKAHEAD
        self.desiredLeftDistance = DESIRED_DISTANCE_LEFT

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
        # self.angle = np.sign(self.angle)*np.min([np.abs(self.angle),np.deg2rad(30)])

        # update values
        self.e1 = self.e0
        self.e2 = self.e1
        self.u1 = self.u0
        self.u2 = self.u1

    def followLeft(self):

        # Estimate future distance
        self.Dtp1 = self.Dt + self.lookahead*np.sin(self.alpha)

        # calculate error
        self.e0 = self.desiredLeftDistance - self.Dtp1

        # Execute PID based on current scan
        self.pid_control()

        # calculate velocity
        self.calcVelocity()

        # Ackermann Msgs
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.angle
        drive_msg.drive.speed = self.velocity
        self.drive_pub.publish(drive_msg)

        # Dtp1 publisher
        #dtp1_msg = Float64()
        #dtp1_msg.data = self.Dtp1
        #self.Dtp1_pub.publish(dtp1_msg)

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

        # save data
        nr_rays = len(data.ranges)
        mid_ray_int = int(nr_rays/2)
        self.midRayRange = data.ranges[mid_ray_int]

    def dynCallback(self, config, level):
        self.Kp = config.Kp
        self.Ki = config.Ki
        self.Kd = config.Kd
        self.N = config.N
        self.desiredLeftDistance = config.DistanceLeft
        return config

    def calcVelocity(self):
        # CHECK VELOCITY ON LAST STEERING ANGLE
        abs_deg_angle = np.abs(np.rad2deg(self.angle))
        # if (abs_deg_angle <= 10):
        #     # Check if distance in front of vehicle is greater than a threshold    
        #     if self.midRayRange > 10:
        #         self.velocity = 7.0
        #     elif 4 <= self.midRayRange <= 10:
        #         self.velocity = 5.0
        #     else:
        #         self.velocity = 2.0
        # else: 
        #     inter_velocity = MAX_VELOCITY*np.abs(np.cos(self.angle))
        #     self.velocity = np.max([1.0,inter_velocity])

        self.velocity = 0.5

        # if (abs_deg_angle <= 10):
        #     # Check if distance in front of vehicle is greater than a threshold    
        #     self.velocity = 3.0
        # else: 
        #     self.velocity = 1.5
            

def main(args):
    rospy.init_node("wall_follow", anonymous=True)
    wf = WallFollow()

    while not rospy.is_shutdown():
        wf.followLeft()
        wf.r.sleep()

if __name__=='__main__':
	main(sys.argv)

