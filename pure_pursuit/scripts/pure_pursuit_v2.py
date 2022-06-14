#!/usr/bin/env python3
import sys
import math
import numpy as np

from visualization_helpers import *
from scipy.interpolate import splprep, splev

#ROS Imports
import rospy
import tf
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker


from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler 

import csv
import os 

from skimage import io, morphology, img_as_ubyte
HZ = 100
TS = 1.0/HZ

class pure_pursuit:

    def __init__(self):
        self.max_velocity = 3.2 # m/s
        self.min_velocity=0.6
        self.velocity = 1.0 # m/s
        self.goal = 0
        #self.read_waypoints()
        self.path_available=False
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        map_topic = '/map'
        odom_topic = '/odom'
        path_topic = '/path'
        wp_vis_pub_topic = '/wp_vis_pub'
        self.r = rospy.Rate(HZ)
        self.proportional_control=0.8
        self.min_lookahead = 1.0
        self.max_lookahead = 3
        self.steering_angle_bound= 0.4
        self.steering_angle_factor=0.7
        self.speed_factor=1.50

        self.lookahead_method='none'

        self.path_pub = rospy.Subscriber(path_topic,Path,self.traj_callback,queue_size=1) # ... todo
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.pose_callback, queue_size=1)
        #self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1) # optional
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)
        self.wp_vis_pub = rospy.Publisher(wp_vis_pub_topic,Marker,  queue_size=1)
        #Publisher for the goal point
        #self.goal_pub = rospy.Publisher('/waypoint/goal', Point, queue_size=1)


        # Import waypoints.csv into a list (path_points)
    def read_waypoints(self,traj_msg):
        filename=[]
        dirname  = os.path.dirname(__file__)
            #filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
            #filename='/home/musaup/Documents/catkin_ws/src/f110-fall2018-skeletons/labs/wall_following/logs/pure-pursuit-wp-2019-04-07-22-39-51.csv'
            #filename='/home/musaup/Documents/catkin_ws/src/f110-fall2018-skeletons/labs/wall_following/logs/pure-pursuit-wp-2019-04-08-02-28-24.csv'
            #filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')

        with open(filename) as f:
                path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = [float(pt.pose.position.x) for pt in traj_msg.poses]
        self.path_points_y   = [float(pt.pose.position.y) for pt in traj_msg.poses]
        self.path_points_w   = [float(pt.pose.position.z) for pt in traj_msg.poses]

        self.dist_arr= np.zeros(len(self.path_points_y))
    
    def wp_preview_trajectory(self):
        for cur_idx in range(0,self.path):
            x_w = self.path[cur_idx, 0]
            y_w = self.path[cur_idx, 1]
            self.publish_waypoint_vis_msg(x_w,y_w)


        # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

        # Input data is PoseStamped message from topic /pf/viz/inferred_pose.
        # Runs pure pursuit and publishes velocity and steering angle.
    def traj_callback(self, traj_msg):
        """
        """
        self.path = np.array([[pt.pose.position.x, pt.pose.position.y] for pt in traj_msg.poses])
        self.path = self.path[::-1]
        self.path_available=True
        #if len(traj_msg) == 0:
        #    rospy.logwarn("Message empty")
        #    self.path_available=True
        #for iy, ix in np.ndindex(self.path.shape):
            #print('paht:',self.path[iy, ix])
        
        #self.path = np.concatenate(([self.pos], self.path))
        # self.s = np.array([pt.positions[0] for pt in traj_msg.points])
        # self.vel = np.array([pt.velocities[0] for pt in traj_msg.points])
        # self.acc = np.array([pt.accelerations[0] for pt in traj_msg.points])
        # self.curv = np.array([pt.effort[0] for pt in traj_msg.points])     

    def find_cur_idx(self):
        #position = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
        point_dist = np.linalg.norm(self.path[:, 0:2] - self.position, axis=1)
        cur_idx = np.argmin(point_dist)
        return cur_idx

    def compute_lookahead(self, cur_idx, method='curvature'):
        if method == 'velocity':
            # Velocity-Based Lookahead
            min_vel = np.min(self.vel)
            max_vel = np.max(self.vel)
            lookahead = np.interp(self.vel[cur_idx],
                                  np.array([min_vel, max_vel]),
                                  np.array([self.min_lookahead, self.max_lookahead]))
        elif method == 'curvature':
            # Curvature-Based Lookahead
            min_curv = np.min(abs(self.curv))
            max_curv = np.max(abs(self.curv))
            lookahead = np.interp(self.curv[cur_idx],
                                  np.array([min_curv, max_curv]),
                                  np.array([self.min_lookahead, self.max_lookahead]))
        else:
            # Constant Lookahead
            lookahead = self.min_lookahead
        return lookahead           

    def get_cur_waypoint(self, cur_idx, lookahead):
        #position = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
        position = self.position
        point_dist = np.linalg.norm(self.path[:, 0:2] - position, axis=1)
        while True:
            cur_idx = (cur_idx + 1) % np.size(self.path, axis=0)
            dis_diff = point_dist[cur_idx] - lookahead
            if dis_diff < 0:
                continue
            elif dis_diff > 0:
                x_w = np.interp(lookahead,
                                np.array([point_dist[cur_idx - 1], point_dist[cur_idx]]),
                                np.array([self.path[cur_idx - 1, 0], self.path[cur_idx, 0]]))
                y_w = np.interp(lookahead,
                                np.array([point_dist[cur_idx - 1], point_dist[cur_idx]]),
                                np.array([self.path[cur_idx - 1, 1], self.path[cur_idx, 1]]))
                break
            else:
                x_w = self.path[cur_idx, 0]
                y_w = self.path[cur_idx, 1]
                break

        return x_w, y_w

    def transform_point(self, goalx, goaly):
        """
        World frame to vehicle frame
        """
        quaternion = self.quaternion

        rot_b_m = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        trans_m = np.array([self.position_x, self.position_y, self.position_z])
        tform_b_m = np.zeros((4, 4))
        tform_b_m[:3, :3] = rot_b_m
        tform_b_m[:3, 3] = trans_m
        tform_b_m[-1, -1] = 1

        goal_m = np.array([[goalx], [goaly], [0], [1]])
        goal_b = (np.linalg.inv(tform_b_m).dot(goal_m)).flatten()

        return goal_b[0], goal_b[1]        

    def compute_steering_angle(self, y_goal_b, lookahead):
        """
        Curvature calculatio:
            Return: desired_angle for trajectory
            param: y_goal_b
            param :lookahead
        """
        y = y_goal_b
        curvature = (2 * y) / lookahead ** 2
        desired_angle = curvature * self.proportional_control 
        return desired_angle


    def publish_drive_msg(self, desired_angle, speed):
        """
        """
        # Compute Control Input
        angle = np.clip(desired_angle,
                        -self.steering_angle_bound,self.steering_angle_bound)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.steering_angle_factor * angle
        drive_msg.drive.speed = self.speed_factor * speed
        self.drive_pub.publish(drive_msg)
        return 0

    def publish_waypoint_vis_msg(self, x, y):
        """
        """
        msg = wp_vis_msg([x, y], rospy.Time.now())
        self.wp_vis_pub.publish(msg)

    def pose_callback(self, odom_msg):
        """
        """
        # Save Odometry
        self.pos = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        self.position = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
        self.quaternion = [odom_msg.pose.pose.orientation.x,
                      odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w]
        self.position_x=odom_msg.pose.pose.position.x
        self.position_y= odom_msg.pose.pose.position.y
        self.position_z= odom_msg.pose.pose.position.z
        # Identify current index position on map

    def update_follow_trajectory(self):
        if self.path_available==False:
            return
        else:
            pass
            #self.wp_preview_trajectory()
            #print('start cur_idx')
            cur_idx = self.find_cur_idx()

            # Obtain appropriate lookahead
            lookahead = self.compute_lookahead(cur_idx, self.lookahead_method)
            # get waypoint to follow based on new lookahead
            x_w, y_w = self.get_cur_waypoint(cur_idx, lookahead)
            # Transform goal point to vehicle frame of reference
            goal_x_body, goal_y_body = self.transform_point( x_w, y_w)
            # Calculate curvature/steering angle
            desired_angle = self.compute_steering_angle(goal_y_body, lookahead)
            desired_speed = self.compute_speed(desired_angle)
            # Publish drive message, don't forget to limit the steering angle.
            #self.publish_drive_msg(desired_angle, self.vel[cur_idx]) self.VELOCITY
            self.publish_drive_msg(desired_angle, desired_speed)
            self.publish_waypoint_vis_msg(x_w, y_w)    



    def odom_callback(self, odom_msg):
        qx=odom_msg.pose.orientation.x
        qy=odom_msg.pose.orientation.y
        qz=odom_msg.pose.orientation.z
        qw=odom_msg.pose.orientation.w
        self.pos = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        # Break if no trajectory

        quaternion = (qx,qy,qz,qw)
        euler   = euler_from_quaternion(quaternion)
        yaw     = euler[2] 

        x = odom_msg.pose.position.x
        y = odom_msg.pose.position.y
        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

            ## finding the distance of each way point from the current position 

        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(x,y))

            ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)

        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]

            ##finding the goal point which is the last in the set of points less than the lookahead distance
            ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
            
        for idx in goal_arr:
            v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                 self.goal = idx
                 # print(self.goal)
                 break

            ##finding the distance of the goal point from the vehicle coordinatesr

        L = self.dist_arr[self.goal]

            ##Transforming the goal point into the vehicle coordinate frame 

        gvcx = self.path_points_x[self.goal] - x
        gvcy = self.path_points_y[self.goal] - y 
        goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)
        # math: find the curvature and the angle 
        alpha = self.path_points_w[self.goal] - (yaw)
        k = 2 * math.sin(alpha)/L
        angle_i = math.atan(k*0.4)
        angle = angle_i*2
        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

        self.set_speed(angle)
            #self.const_speed(angle)

            #publish the goal in the vehicle coordinates. 
        goalPoint = Point(float(goal_x_veh_coord),float(goal_y_veh_coord),float(angle));
        self.goal_pub.publish(goalPoint)


    # USE THIS FUNCTION IF CHANGEABLE SPEED IS NEEDED
    def compute_speed(self,angle):
        if (abs(angle)>0.2018):
            self.min_lookahead = 0.9
            self.velocity -= 0.1#0.7
            if self.min_velocity - self.velocity >= 0.0:
                self.velocity -= 0.2#0.7
                if self.velocity<=self.min_velocity:
                    self.velocity= self.min_velocity
        else:
            self.min_lookahead = 1.3
            if self.max_velocity - self.velocity >= 0.5:
                self.velocity += 0.1
        return self.velocity

        # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

def main(args):
    rospy.init_node("pure_pursuit_node", anonymous=True)
    rfgs = pure_pursuit()
    cur_idx=0
    while not rospy.is_shutdown():
        rfgs.update_follow_trajectory()
        #rfgs.publish_drive_msg(0,1)
        rfgs.r.sleep()
        # if rfgs.path_available==True:
        #     x_w = rfgs.path[cur_idx, 0]
        #     y_w = rfgs.path[cur_idx, 1]
        #     rfgs.publish_waypoint_vis_msg(x_w,y_w)
        #     rfgs.r.sleep()
        #     if cur_idx== 1545:
        #         cur_idx=0
        #     cur_idx+=1

if __name__ == '__main__':
    main(sys.argv)
