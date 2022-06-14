#!/usr/bin/env python3
from __future__ import print_function
from distutils.log import info
from email.charset import SHORTEST
from pickle import FALSE, TRUE
import sys
import math
from matplotlib.pyplot import xcorr
import numpy as np
import heapq
import scipy.interpolate as si

import csv
import os 
import rospkg

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
#get the path for this paackage
package_path=rospack.get_path('pure_pursuit')

#ROS Imports

import rospy
import tf
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Point32
from visualization_msgs.msg import Marker
from tf import transformations # rotation_matrix(), concatenate_matrices()


from skimage import io, morphology, img_as_ubyte
from skimage.util import invert
import  rviz_tools
from skimage.morphology import disk ,binary_closing,skeletonize # noqa




HZ = 100
SHORTEST_PATH=False
READ_ROUTE=True


class planner:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        map_topic = '/map'
        odom_topic = '/odom'
        path_topic = '/path'

         # rosrate
        self.r = rospy.Rate(HZ)
        self.n_safety=7
        self.map_binary= np.zeros( (2000, 2000) , dtype=np.int8) 
        self.skeleton_binary = np.zeros( (2000, 2000) , dtype=np.int8) 
        self.driveable= np.ones( (2000, 2000) , dtype=np.int64)
        self.centerline= np.ones( (2000, 2000) , dtype=np.int64)  
        self.center= np.ones( (2000, 2000) , dtype=np.int64) 

        self.driveable_ps= np.ones( (2000, 2000) , dtype=np.int64) 
        self.route_pre=[]
        self.path_pub = rospy.Publisher(path_topic,Path,queue_size=1) # ... todo
        if READ_ROUTE==True:
            self.read_waypoints()
            
        self.pos_x = 0
        self.pos_y = 0
        self.grid_x =  0
        self.grid_y =  0
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        self.markers_pub = rospy.Publisher('visualization_marker_array', MarkerArray,queue_size=1)


    def read_waypoints(self):
        """
        Import waypoints.csv into a list (path_points)
        """
        filename=[]
        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/f1_wide_log_minCurv.csv')
        CSVData = open(filename)
        self.route_pre = np.genfromtxt(CSVData, delimiter=",")
        self.publish_waypoints(self.route_pre)
        print('finis reading')

    def floodfill_stack(self,matrix,x,y,n_safety):     
        self.markerArray = MarkerArray()
        selected_pixel_sign = matrix[x,y]
        stack=[(1000,1000)]
        print(selected_pixel_sign)
        m,n = matrix.shape
        while stack:
            x,y = stack.pop()
            if  0 <= x < m-1 and 0 <= y < n-1 and matrix[x,y] == selected_pixel_sign and self.driveable[x,y] ==1: # check bounds and height
                #dist=self.check_safety_distance(self.racebounds,x,y) 
                #print('dist',dist)
                #if dist>n_safety:
                    self.driveable[x,y] = 0 # set the value
                    stack.append((x-1,y-1)) # add all directions to the stack, we check later
                    stack.append((x,y-1))
                    stack.append((x+1,y-1))
                    stack.append((x-1,y))
                    stack.append((x+1,y))
                    stack.append((x-1,y+1))
                    stack.append((x,y+1))
                    stack.append((x+1,y+1))
        return self.set_startLine()
    
    def floodfill_center(self,matrix,x,y,n_safety):     
        self.markerArray = MarkerArray()
        selected_pixel_sign = matrix[x,y]
        stack=[(999,999)]
        print(selected_pixel_sign)
        m,n = matrix.shape
        while stack:
            x,y = stack.pop()
            if  0 <= x < m-1 and 0 <= y < n-1 and matrix[x,y] == selected_pixel_sign and self.centerline[x,y] ==1: # check bounds and height
                #dist=self.check_safety_distance(self.racebounds,x,y) 
                #print('dist',dist)
                #if dist>n_safety:
                    self.centerline[x,y] = 0 # set the value
                    stack.append((x-1,y-1)) # add all directions to the stack, we check later
                    stack.append((x,y-1))
                    stack.append((x+1,y-1))
                    stack.append((x-1,y))
                    stack.append((x+1,y))
                    stack.append((x-1,y+1))
                    stack.append((x,y+1))
                    stack.append((x+1,y+1))
        return self.add_startLine()
    
    def add_startLine(self):
        for  ix in range(-5, 5): 
          if self.centerline[self.grid_x+ix, self.grid_y]==1:
              pass
          else:
             self.centerline[self.grid_x+ix,self.grid_y]=1
             self.centerline[self.grid_x+ix,self.grid_y-1]=1

        return self.centerline
    
    def set_startLine(self):
        for  ix in range(-30, 30): 
            if self.driveable[self.grid_x+ix, self.grid_y]==1:
                pass
            else:
                self.driveable[self.grid_x+ix,self.grid_y]=1
                self.driveable[self.grid_x+ix,self.grid_y-1]=1

        return self.driveable
    
    def check_safety_distance(self,arr, x1, y1):
        """ 
        Return euclidean distance between points p and q
        assuming both to have the same number of dimensions
        """
        min_dist = np.zeros(len(arr))
        for i in range(len(arr)):
            x2 = arr[i][0]
            y2 = arr[i][1]
            min_dist[i] = math.dist([x1, y1], [x2,y2])
        return min(min_dist)

    def visualize_maps(self,data):
        self.markerArray = MarkerArray()
        i=0
        for iy, ix in np.ndindex(data.shape):
            if data[iy, ix]==0:
                i+=1
                mark = Marker()
                mark =self.visualize_rectangle(i,ix-self.grid_x,iy-self.grid_y)
                self.markerArray.markers.append(mark)
        self.markers_pub.publish(self.markerArray)
        print('end ff and publish')
    

    def visualize_rectangle(self,i,x,y):
          # Rectangle Marker
         # Create the Rviz Marker Publisher
        mark = Marker()
        mark.header.frame_id = 'map'
        mark.header.stamp=rospy.Time.now()
        mark.lifetime =  rospy.Duration(0) # 0 = Marker never expires
        mark.pose.position.x = x*0.05
        mark.pose.position.y = y*0.05
        mark.pose.position.z = -0.10
        mark.pose.orientation.y = 0.0
        mark.pose.orientation.w = 1.0
        mark.scale.x = 0.05
        mark.scale.y = 0.05
        mark.scale.z = 0.01
        mark.color.a = 0.3
        mark.color.r = 1.0
        mark.color.g = 0.5
        mark.color.b = 0.1
        mark.id=i
        mark.ns = "Rectangle" # unique ID
        mark.action = mark.ADD
        mark.type = mark.CUBE
        return mark

    def publish_waypoints(self,route):
            """
            Publish the ROS message containing the waypoints
            """
            self.current_route=route
            msg = Path()
            if READ_ROUTE==False:
                x=1
                y=0
            else:
                s_m=0
                x=1
                y=2
                # x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
                
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            if self.current_route is not None:
                for wp in (range(0,len(self.current_route)-1)): 
                    if READ_ROUTE==True:        
                        first_locationX = self.current_route[wp][x]*1
                        first_locationY = self.current_route[wp][y]*1
                        first_locationZ = 0
    
                        second_locationX = self.current_route[wp+1][x]*1
                        second_locationY = self.current_route[wp+1][y]*1
                        second_locationZ = 0
                    else:
                        first_locationX = self.current_route[wp][x]*0.05-50
                        first_locationY = self.current_route[wp][y]*0.05-50
                        first_locationZ = 0
    
                        second_locationX = self.current_route[wp+1][x]*0.05-50
                        second_locationY = self.current_route[wp+1][y]*0.05-50
                        second_locationZ = 0

                    pose = PoseStamped()
                    pose.pose.position.x = first_locationX
                    pose.pose.position.y = first_locationY
                    pose.pose.position.z = first_locationZ
                    dX = first_locationX - second_locationX
                    dY = first_locationY - second_locationY
                    dZ = first_locationZ - second_locationZ

                    yaw=math.atan2(dY, dX)
                    pitch = math.atan2(math.sqrt(dZ * dZ + dX * dX), dY) + math.pi

                    quaternion = quaternion_from_euler(0, 0, -math.radians(yaw))
                    pose.pose.orientation.x = quaternion[0]
                    pose.pose.orientation.y = quaternion[1]
                    pose.pose.orientation.z = quaternion[2]
                    pose.pose.orientation.w = quaternion[3]
                    msg.poses.append(pose)

            self.path_pub.publish(msg)
            rospy.loginfo("Published {} waypoints.".format(len(msg.poses))) 


    def map_callback(self, data):
        """ Process the map to pre-compute the path to follow and publish it
        """
        rospy.loginfo("map info from OccupancyGrid message:\n%s", data.info)

        occupied_thresh = 0.65
        free_thresh = 0.2
        footprint = disk(self.n_safety)

        map_data = np.asarray(data.data).reshape((data.info.width, data.info.height)) # parse map data into 2D numpy array
        map_data = morphology.dilation(map_data,footprint)
        map_normalized = map_data / np.amax(map_data.flatten()) # normalize map
        self.map_binary = map_normalized < (occupied_thresh) # make binary occupancy map
        
        self.pos_x = data.info.origin.position.x # pos in -50 to 50
        self.pos_y = data.info.origin.position.y
  
        self.grid_x =  int(- self.pos_x / data.info.resolution )# in pixel
        self.grid_y =  int(- self.pos_y / data.info.resolution ) # in pixel
        
        self.map_binary = np.rot90(np.flip(self.map_binary, 1), 0) # flip and rotate for rendering as image in correct direction
        self.map_binary = np.rot90(np.flip(self.map_binary, 1), 0) # flip and rotate for rendering as image in correct direction
        self.skeleton_binary = binary_closing(skeletonize(self.map_binary),footprint)
        #mask = flood(self.skeleton_binary, (1000, 1000), tolerance=0)
        #self.skeleton_pt=self.skeleton_binary *0.05-50
        #io.imsave('~/catkin_ws/src/pure_pursuit/maps/skel_test.png', img_as_ubyte(self.skeleton_binary), check_contrast=False) # save image, just to show the content of the 2D array for debug purposes

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)

        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:# array bound y walls
                    continue
            else:# array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
 
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor)) 

def bspline(cv, n=100, degree=5, periodic=False):
    """ Calculate n samples on a bspline
        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
    """
    cv = np.asarray(cv)
    count = cv.shape[0]

    # Closed curve
    if periodic:
        kv = np.arange(-degree,count+degree+1)
        factor, fraction = divmod(count+degree+1, count)
        cv = np.roll(np.concatenate((cv,) * factor + (cv[:fraction],)),-1,axis=0)
        degree = np.clip(degree,1,degree)

    # Opened curve
    else:
        degree = np.clip(degree,1,count-1)
        kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

    # Return samples
    max_param = count - (degree * (1-periodic))
    spl = si.BSpline(kv, cv, degree)
    return spl(np.linspace(0,max_param,n))    
    

def main(args):
    rospy.init_node("planner_node", anonymous=True)
    rfgs = planner()
    rospy.sleep(2) # Sleeps for 1 sec
    fertig=False
    start = (rfgs.grid_x,rfgs.grid_x) #(| y) (- x )
    goal = (rfgs.grid_x,990) #y -  
    markers = rviz_tools.RvizMarkers('map', 'visualization_marker')
    
    if READ_ROUTE==False: 
        if SHORTEST_PATH:
            rfgs.driveable= rfgs.floodfill_stack(rfgs.map_binary,0,0,0)
            rospy.sleep(15) # Sleeps for 1 sec  
            route = astar(rfgs.driveable, start, goal)
            route=bspline(route,n=200,degree=120,periodic=True) 
        else:
            rospy.sleep(15) # Sleeps for 1 sec   
            rfgs.centerline= rfgs.floodfill_center(rfgs.skeleton_binary,999,999,0)
            route = astar(rfgs.centerline, start, goal)
        rospy.sleep(1) # Sleeps for 1 sec
        rfgs.publish_waypoints(route)
        rospy.sleep(1) # Sleeps for 1 se


    while not rospy.is_shutdown():          

        if fertig==False: 
            #rfgs.visualize_maps(rfgs.centerline)
            if READ_ROUTE==True: 
                rospy.sleep(4) # Sleeps for 1 sec
                route=bspline(rfgs.route_pre,n=500,degree=3,periodic=True) 
                rfgs.publish_waypoints(route)
            rospy.sleep(4) # Sleeps for 1 sec
            # Publish a path using a list of ROS Point Msgs
            print('fertig')
            fertig=True
        
        rospy.sleep(0.01) # Sleeps for 1 sec

if __name__ == '__main__':
    main(sys.argv)