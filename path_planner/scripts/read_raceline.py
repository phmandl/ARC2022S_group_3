#!/usr/bin/env python3
import sys
import math
import numpy as np
import csv
import os 

#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Usefull stuff
from scipy import interpolate
import matplotlib.pyplot as plt

class planner:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        path_topic = '/path'
        mpc_topic = '/mpc_path'
        odom_topic = '/odom'

        # rosrate
        self.Hz = 100
        self.r = rospy.Rate(self.Hz)

        # Publisher
        self.path_pub = rospy.Publisher(path_topic,Path,queue_size=1)
        self.mpc_path_pub = rospy.Publisher(mpc_topic,Path,queue_size=1)

        # Subscriber
        self.odom_sub = rospy.Subscriber(odom_topic,Odometry,self.odom_callback)

        # CSV file
        self.dirname = os.path.dirname(__file__)
        self.filename = os.path.join(self.dirname, '../waypoints/f1_wide_log_minCurv.csv')
        
        # MPC Stuffy
        self.Tl = 1.0 # Endtime
        self.Ts = 0.05 # MPC sample time
        self.Np = int(self.Tl/self.Ts) # Prediction-Horizon
        self.horizon = np.linspace(0,self.Tl - self.Ts,self.Np)
        self.index_position = 0 # Vehicle position on path

        # PATHY stuffy
        self.ds = 0.05 # bspline interpolation on 5cm grid
        self.plot_flag = False
        self.route = []
        self.s_int = []
        self.x_int = []
        self.y_int = []
        self.yaw_int = []
        self.kappa_int = []
        self.vx_int = []
        self.ax_int = []

        self.path_size = 0

        # Vehicle stuffy
        self.x_pos = 0.0
        self.y_pos = 0.0

    def read_waypoints(self):
        """
        Import waypoints.csv into a list (path_points)
        """
        CSVData = open(self.filename)
        self.route = np.genfromtxt(CSVData, delimiter=",")
        rospy.loginfo("Planner: Read CSV file!")
    
    def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """                
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        
        # planner indexes
        #s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
        path_s = self.route[:,0]
        path_x = self.route[:,1]
        path_y = self.route[:,2]
        path_yaw = self.route[:,3] + np.deg2rad(90) # rotate by 90 degrress --> TUM defines zero wiht y-axis not x-axis
        path_kappa = self.route[:,4]
        path_vx = self.route[:,5]/2
        path_ax = self.route[:,6]

        # Interpolate path
        self.s_int = np.arange(path_s[0],path_s[-1],self.ds)
        self.x_int = self.bspline_interpolation(path_s,self.s_int,path_x,self.plot_flag)
        self.y_int = self.bspline_interpolation(path_s,self.s_int,path_y,self.plot_flag)
        self.yaw_int = self.bspline_interpolation(path_s,self.s_int,path_yaw,self.plot_flag)
        self.kappa_int = self.bspline_interpolation(path_s,self.s_int,path_kappa,self.plot_flag)
        self.vx_int = self.bspline_interpolation(path_s,self.s_int,path_vx,self.plot_flag)
        self.ax_int = self.bspline_interpolation(path_s,self.s_int,path_ax,self.plot_flag)

        # Path size
        self.path_size = len(self.s_int)

        # Build poses!
        for wp in range(0,len(self.s_int)):        
            pose = PoseStamped()
            pose.pose.position.x = self.x_int[wp]
            pose.pose.position.y = self.y_int[wp]
            # pose.pose.position.z = self.vx_int[wp]
            pose.pose.position.z = 0

            quaternion = quaternion_from_euler(0, 0, self.yaw_int[wp])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            # append pose to poses
            msg.poses.append(pose)

        rospy.sleep(1)
        self.path_pub.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses))) 

    def calc_mpc_vectors(self):
        # calc euclidian norm
        dx = (self.x_int - self.x_pos)
        dy = (self.y_int - self.y_pos)
        dist = np.sqrt(dx**2 + dy**2)
        self.index_position = dist.argmin()

        # Calculate with vref forward in time and find end-index
        dt = self.ds/self.vx_int
        time = np.cumsum(dt)
        time = np.insert(time,0,0)
        index_Tl = np.abs(time - self.Tl).argmin()

        # Build desired vecotors: vref_mpc, x_mpc, y_mpc
        index_vec = np.zeros(int(self.Np),dtype = int)
        for idx, val in enumerate(self.horizon):
            minIdx = np.abs(time - val).argmin()
            last_index = self.path_size - 1
            if (minIdx + self.index_position > last_index):
                index_vec[idx] = minIdx - (last_index - self.index_position) # detect overflow of path
            else:
                index_vec[idx] = minIdx + self.index_position

        # Vectors
        x_mpc = self.x_int[index_vec]
        y_mpc = self.y_int[index_vec]
        yaw_mpc = self.yaw_int[index_vec]
        kappa_mpc = self.kappa_int[index_vec]
        vref_mpc = self.vx_int[index_vec]
        ax_mpc = self.ax_int[index_vec]

        # Calculate lateral error e1 and yaw angle error e2
        # dx[self.index_position]
        dv = np.zeros(3)
        dv[0] = dx[self.index_position]
        dv[1] = dy[self.index_position]
        dv = dv/np.linalg.norm(dv) 
        
        ds = np.zeros(3)
        ds[0] = x_mpc[1] - x_mpc[0]
        ds[1] = y_mpc[1] - y_mpc[0]
        ds = ds/np.linalg.norm(ds)

        cross_product = np.cross(dv, ds)

        # Calc errors
        e1 = np.sign(cross_product[2])*dist[self.index_position] # approximation --> lateral error is only dy
        e2 = self.yaw - yaw_mpc[0] 
        print(e1)

        # Publish
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        # Build poses!
        for wp in range(0,self.Np):        
            pose = PoseStamped()
            pose.pose.position.x = x_mpc[wp]
            pose.pose.position.y = y_mpc[wp]
            pose.pose.position.z = vref_mpc[wp]

            # USE ORIENTATION TO DELIVER OUTHER INFORMATION
            # --------------------------------------------
            pose.pose.orientation.x = ax_mpc[wp]        # long. acceleration
            pose.pose.orientation.y = kappa_mpc[wp]     # curvature
            pose.pose.orientation.z = e1                 # error e1
            pose.pose.orientation.w = e2                 # error e2

            # append pose to poses
            msg.poses.append(pose)

        self.mpc_path_pub.publish(msg)

    def odom_callback(self,msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.yaw_rate = msg.twist.twist.angular.z

        rot_x = msg.pose.pose.orientation.x
        rot_y = msg.pose.pose.orientation.y
        rot_z = msg.pose.pose.orientation.z
        rot_w = msg.pose.pose.orientation.w
        euler_angles = euler_from_quaternion([rot_x,rot_y,rot_z,rot_w])
        self.yaw = euler_angles[2]

    def bspline_interpolation(self,s_old,s_new,data,plot_flag):
        tck = interpolate.splrep(s_old, data, s=0)
        ynew = interpolate.splev(s_new, tck, der=0)

        if plot_flag == True:                
            plt.figure()
            plt.plot(s_old, data, 'x', s_new, ynew, 'b')
            # plt.legend(['Linear', 'Cubic Spline', 'True'])
            # plt.axis([-0.05, 6.33, -1.05, 1.05])
            plt.title('Cubic-spline interpolation')
            plt.show()

        return ynew


def main(args):
    rospy.init_node("planner_node", anonymous=True)
    rfgs = planner()
    is_published = False

    # Publish waypoints
    while not rospy.is_shutdown(): 
        if is_published == False:
            rfgs.read_waypoints()
            rfgs.publish_waypoints()
            rospy.loginfo("Planner: Published Waypoints")
            is_published = True
        else:
            rfgs.calc_mpc_vectors()
            rfgs.r.sleep()

if __name__ == '__main__':
    main(sys.argv)