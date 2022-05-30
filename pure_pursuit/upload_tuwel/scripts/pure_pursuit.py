#!/usr/bin/env python3
import sys
import numpy as np
from numpy import linalg as la
# import csv
# import os 

# from visualization_helpers import *
from scipy.interpolate import splprep, splev

#ROS Imports
import rospy
import tf
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler 


HZ = 100
TS = 1.0/HZ

class pure_pursuit:

    def __init__(self):
        self.max_velocity = 3.0 # m/s
        self.min_velocity=0.5
        self.velocity = 1.0 # m/s
        self.goal = 0
        #self.read_waypoints() # if we would do some global optimal path
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
        self.min_lookahead = 0.5
        self.max_lookahead = 1.2
        self.lookahead = self.min_lookahead
        self.angle=0
        self.steering_angle_bound= 0.41
        self.steering_angle_factor=1.0
        self.lookahead_angle=0.2 #0.2018
        self.speed_factor=1.00

        self.lookahead_method='angle'

        self.path_pub = rospy.Subscriber(path_topic,Path,self.traj_callback,queue_size=1) # ... todo
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        #self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1) # optional
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)
        self.wp_vis_pub = rospy.Publisher(wp_vis_pub_topic,Marker,  queue_size=1)
        #Publisher for the goal point
        #self.goal_pub = rospy.Publisher('/waypoint/goal', Point, queue_size=1)


    def read_waypoints(self,traj_msg):
        """
        Import waypoints.csv into a list (path_points)
        """
        filename=[]
        dirname  = os.path.dirname(__file__)
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


        
    def dist(self, p1, p2):
        """
        # Computes the Euclidean distance between two 2D points p1 and p2.
        """
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


    def traj_callback(self, traj_msg):
        """
        """
        self.path = np.array([[pt.pose.position.x, pt.pose.position.y] for pt in traj_msg.poses])
        self.path = self.path[::-1]
        self.path_available=True
        # Compute the velocity in path depending on the angle
        #if len(traj_msg) == 0:
        #    rospy.logwarn("Message empty")
        #    self.path_available=True
        #for iy, ix in np.ndindex(self.path.shape):
            #print('paht:',self.path[iy, ix])
        # self.vel = np.array([pt.velocities[0] for pt in traj_msg.points])
  

    def find_cur_idx(self):
        #position = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
        point_dist = np.linalg.norm(self.path[:, 0:2] - self.position, axis=1)
        cur_idx = np.argmin(point_dist)
        return cur_idx

    def compute_lookahead(self, cur_idx, method='angle'):
        """
        World frame to vehicle frame
        """
        if method == 'velocity':
            # Velocity-Based Lookahead
            min_vel = np.min(self.vel)
            max_vel = np.max(self.vel)
            self.lookahead = np.interp(self.vel[cur_idx],
                                  np.array([min_vel, max_vel]),
                                  np.array([self.min_lookahead, self.max_lookahead]))
        elif method == 'curvature':
            # Curvature-Based Lookahead
            min_curv = np.min(abs(self.curv))
            max_curv = np.max(abs(self.curv))
            self.lookahead = np.interp(self.curv[cur_idx],
                                  np.array([min_curv, max_curv]),
                                  np.array([self.min_lookahead, self.max_lookahead]))
        elif method == 'angle':
            # Angle-Based Lookahead
            # print('angle',self.angle)
            if (abs(self.angle)>self.lookahead_angle): #0.2018
                self.lookahead -= 0.3#0.7
                if self.lookahead<=self.min_lookahead:
                    self.lookahead= self.min_lookahead
            else:
                self.lookahead += 0.1
                if self.lookahead>=self.max_lookahead:
                    self.lookahead= self.max_lookahead
                     
        else:
            # Constant Lookahead
            self.lookahead = self.min_lookahead
        # print('lookahead',self.lookahead)    
        return self.lookahead           

    def get_cur_waypoint(self, cur_idx, lookahead):
        """
        World frame to vehicle frame
        """
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

    # def publish_waypoint_vis_msg(self, x, y):
        # """
        # """
        # msg = wp_vis_msg([x, y], rospy.Time.now())
        # self.wp_vis_pub.publish(msg)

    def odom_callback(self, odom_msg):
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
            self.angle = self.compute_steering_angle(goal_y_body, lookahead)
            desired_speed = self.compute_speed(self.angle)
            # Publish drive message, don't forget to limit the steering angle.
            #self.publish_drive_msg(desired_angle, self.vel[cur_idx]) self.VELOCITY
            self.publish_drive_msg(self.angle, desired_speed)
            # self.publish_waypoint_vis_msg(x_w, y_w)    


    def compute_speed(self,angle):
        if (abs(angle)>self.lookahead_angle):
            if self.min_velocity - self.velocity <= 0.0:
                self.velocity -= 0.3#0.7
                if self.velocity<=self.min_velocity:
                    self.velocity= self.min_velocity
        else:
            if self.max_velocity - self.velocity >= 0.5:
                self.velocity += 0.15
                if self.max_velocity<=self.velocity:
                    self.velocity= self.max_velocity
        return self.velocity


def main(args):
    rospy.init_node("pure_pursuit_node", anonymous=True)
    rfgs = pure_pursuit()
    cur_idx=0
    while not rospy.is_shutdown():
        rfgs.update_follow_trajectory()
        rfgs.r.sleep()
        # DEBUG:
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
