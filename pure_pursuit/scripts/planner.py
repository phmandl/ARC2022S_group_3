#!/usr/bin/env python3
# from __future__ import print_function
# from distutils.log import info
# from pickle import FALSE, TRUE
import sys
import math
# from matplotlib.pyplot import xcorr
import numpy as np
import heapq
import scipy.interpolate as si

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


from std_msgs.msg import Header, ColorRGBA, Float64MultiArray
from geometry_msgs.msg import Point, Point32
from visualization_msgs.msg import Marker
from tf import transformations # rotation_matrix(), concatenate_matrices()


from skimage import io, morphology, img_as_ubyte
# import  rviz_tools
from skimage.morphology import disk  # noqa

import matplotlib.pyplot as plt

HZ = 100



class planner:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        map_topic = '/map'
        odom_topic = '/odom'
        path_topic = '/path'
        path_speed = '/path_speed'
        path_curv = '/path_curv'
        path_len = '/path_len'
        path_yaw = '/path_yaw'

         # rosrate
        self.r = rospy.Rate(HZ)
        self.n_safety=9
        self.map_binary= np.zeros( (2000, 2000) , dtype=np.int8)  
        self.driveable= np.ones( (2000, 2000) , dtype=np.int64) 
        self.driveable_ps= np.ones( (2000, 2000) , dtype=np.int64) 
        self.racebounds=[]


        self.pos_x = 0
        self.pos_y = 0
        self.grid_x =  0
        self.grid_y =  0
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        self.markers_pub = rospy.Publisher('visualization_marker_array', MarkerArray,queue_size=1)
        self.path_pub = rospy.Publisher(path_topic,Path,queue_size=1,latch=True) # ... todo
        self.path_speed_pub = rospy.Publisher(path_speed,Float64MultiArray,queue_size=1,latch=True)
        self.path_curv_pub = rospy.Publisher(path_curv,Float64MultiArray,queue_size=1,latch=True)
        self.path_len_pub = rospy.Publisher(path_len,Float64MultiArray,queue_size=1,latch=True)
        self.path_yaw_pub = rospy.Publisher(path_yaw,Float64MultiArray,queue_size=1,latch=True)


    def floodfill_stack(self,matrix,x,y,n_safety):     
        self.markerArray = MarkerArray()
        selected_pixel_sign = matrix[x,y]
        stack=[(999,999)]

        for iy, ix in np.ndindex(matrix.shape):
            if matrix[ix, iy]==True:
                continue
            if matrix[ix, iy]==False:
                self.racebounds.append( (ix,iy) )
        m,n = matrix.shape
        while stack:
            x,y = stack.pop()
            if  0 <= x < m-1 and 0 <= y < n-1 and matrix[x,y] == selected_pixel_sign and self.driveable[x,y] ==1: # check bounds and height
                #dist=self.check_safety_distance(self.racebounds,x,y) 
                #print('dist',dist)
                #if dist>n_safety:
                    self.driveable[x,y] = 0 # set the value
                    stack.append((x,y-1))
                    stack.append((x+1,y-1))
                    stack.append((x-1,y))
                    stack.append((x+1,y))
                    stack.append((x-1,y+1))
                    stack.append((x,y+1))
        return self.set_startLine()
    
    def set_startLine(self):
        for  ix in range(-60, 60): 
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

            # for MPC
            s_vec = np.zeros(len(self.current_route))
            x_vec = np.zeros(len(self.current_route))
            y_vec = np.zeros(len(self.current_route))
            z_vec = np.zeros(len(self.current_route))
            yaw_vec = np.zeros(len(self.current_route))
            pitch_vec = np.zeros(len(self.current_route))

            if self.current_route is not None:
                for wp in (range(0,len(self.current_route)-1)):         
                    first_locationX = self.current_route[wp][1]*0.05-50
                    first_locationY = self.current_route[wp][0]*0.05-50
                    first_locationZ = 0
 
                    second_locationX = self.current_route[wp+1][1]*0.05-50
                    second_locationY = self.current_route[wp+1][0]*0.05-50
                    second_locationZ = 0

                    dX = first_locationX - second_locationX
                    dY = first_locationY - second_locationY
                    dZ = first_locationZ - second_locationZ

                    yaw = math.atan2(dY, dX)
                    pitch = math.atan2(math.sqrt(dZ * dZ + dX * dX), dY) + math.pi

                    # Calculate some stuff
                    x_vec[wp] = first_locationX
                    y_vec[wp] = first_locationY
                    z_vec[wp] = first_locationZ
                    yaw_vec[wp] = yaw
                    pitch_vec[wp] = pitch
                    s_vec[wp + 1] = s_vec[wp] + np.sqrt(dX**2 + dY**2 + dZ**2)

                # Last location
                x_vec[wp + 1] = second_locationX
                y_vec[wp + 1] = second_locationY
                z_vec[wp + 1] = second_locationZ

                # Calculate curvature
                [s_big,x_big,y_big,curv_big,vref_big,yaw_big,pitch_big] = self.calc_curvature_vref_interp(s_vec,x_vec,y_vec,yaw_vec,pitch_vec)

                # GEN PATH
                msg = Path()
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()

                msg_speed = Float64MultiArray()
                msg_speed.data = vref_big
                self.path_speed_pub.publish(msg_speed)

                msg_curv = Float64MultiArray()
                msg_curv.data = curv_big
                self.path_curv_pub.publish(msg_curv)

                msg_len = Float64MultiArray()
                msg_len.data = s_big
                self.path_len_pub.publish(msg_len)

                msg_yaw = Float64MultiArray()
                msg_yaw.data = yaw_big
                self.path_yaw_pub.publish(msg_yaw)

                for idx, val in enumerate(s_big):
                    pose = PoseStamped()
                    pose.pose.position.x = x_big[idx]
                    pose.pose.position.y = y_big[idx]
                    pose.pose.position.z = 0

                    quaternion = quaternion_from_euler(0, 0, -math.radians(yaw_big[idx]))
                    pose.pose.orientation.x = quaternion[0]
                    pose.pose.orientation.y = quaternion[1]
                    pose.pose.orientation.z = quaternion[2]
                    pose.pose.orientation.w = quaternion[3]
                    msg.poses.append(pose)

                self.path_pub.publish(msg)
                rospy.loginfo("Published {} waypoints.".format(len(msg.poses))) 

    def calc_curvature_vref_interp(self, s, x, y, yaw, pitch):
        dx_dt = np.gradient(x)
        dy_dt = np.gradient(y)

        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)

        # Curvature
        curvature = (dx_dt * d2y_dt2 - d2x_dt2 * dy_dt) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5

        # Calculate reference velocity
        v_ref = np.zeros(len(curvature))
        for idx, kappa in enumerate(curvature):
            v_ref[idx] = np.sqrt(9.81*0.1/np.abs(kappa))
            v_ref[idx] = np.min((v_ref[idx], 3))
        
        # DEBUG
        print(curvature)
        print(x)
        print(y)
        print(yaw)

        # Interpolate everything!
        dx = 0.05
        s_big = np.arange(s[0],s[-1],dx)
        x_big = np.interp(s_big,s,x)
        y_big = np.interp(s_big,s,y)
        curv_big = np.interp(s_big,s,curvature)
        vref_big = np.interp(s_big,s,v_ref)
        yaw_big = np.interp(s_big,s,yaw)
        pitch_big = np.interp(s_big,s,pitch)

        return s_big,x_big,y_big,curv_big,vref_big,yaw_big,pitch_big

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
        #io.imsave('~/catkin_ws/src/pure_pursuit/maps/new_map.png', img_as_ubyte(self.map_binary), check_contrast=False) # save image, just to show the content of the 2D array for debug purposes

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
    #start = [999, 1010]
    #end = [999, 1190]
    fertig=False
    start = (rfgs.grid_x,rfgs.grid_x) #(| y) (- x )
    goal = (rfgs.grid_x,989) #y -  
    # markers = rviz_tools.RvizMarkers('map', 'visualization_marker')
    rfgs.driveable= rfgs.floodfill_stack(rfgs.map_binary,0,0,0)
    route = astar(rfgs.driveable, start, goal)
    route=bspline(route,n=120,degree=120,periodic=True) 
    rospy.sleep(1) # Sleeps for 1 sec
    path = []
    for i in (range(0,len(route))):
        x = route[i][0]*0.05-50
        y = route[i][1]*0.05-50
        path.append( Point(y,x,0) )
    path.append( Point(route[0][1]*0.05-50,route[0][0]*0.05-50,0) )
    rfgs.publish_waypoints(route)

    while not rospy.is_shutdown():          

        if fertig==False: 
            #rfgs.visualize_maps(rfgs.driveable)
            rospy.sleep(1) # Sleeps for 1 sec
            # Publish a path using a list of ROS Point Msgs
            width = 0.1
            # markers.publishPath(path, 'green', width, 0.0) # path, color, width, lifetime
            print('fertig')
            fertig=True
        
        rospy.sleep(0.01) # Sleeps for 1 sec

if __name__ == '__main__':
    main(sys.argv)
