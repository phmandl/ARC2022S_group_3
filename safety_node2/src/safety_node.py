#!/usr/bin/env python
import rospy

# TODO: import ROS msg types and libraries

import numpy as np
import precompute
from std_msgs.msg import String,Bool
from Ackermann_msgs.msg import AckermannDriveStamped
import fstream
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


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
        # TODO: create ROS subscribers and publishers.
        rospy.Subscriber("odom",Odometry,odom_callback)
        rospy.Subscriber("scan",LaserScan,scan_callback)
        
        brake_bool_pub=rospy.Publisher("brake_bool",Bool,queue_size=1)
        brake_pub=rospy.Publisher("brake",AckermannDriveStamped,queue_size=1)
        
        self.brake_ttc_threshold=rospy.get_param('~brake_ttc_threshold')
        
        scan_beams=rospy.get_param('~scan_beams')
        wheelbase=rospy.get_param('~wheelbase')
        width=rospy.get_param('~width')
        scan_dist_to_base_link=rospy.get_param('~scan_distance_to_base_link')
        scan_fov=rospy.get_param('~scan_field_of_view')
        scan_angle_incr=scan_fov/scan_beams
        
        precompute=precompute(scan_beams, 
		              wheelbase, 
		              width, 
		              scan_distance_to_base_link,
		              -scan_fov/2.0,
		              scan_angle_incr)
		              
        self.cosines,self.car_distances=precompute.cosines_and_car_dist()
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        
        brk=False
        
        if(self.speed!=0):
        	for i in range(scan_msg.ranges.size()):
        		proj_speed=self.speed * self.cosines[i]
        		ttc=(scan_msg.ranges[i]-self.car_distances[i])/proj_speed
        		
        		# TODO: publish brake message and publish controller bool
        		
        		if((ttc<self.brake_ttc_threshold) and (ttc>0)):
        			ack_msg=AckermannDriveStamped()
        			ack_msg.header.stamp=rospy.Time.now()
        			ack_msg.drive.steering_angle=0.0
        			ack_msg.drive.speed=0.0
        			brake_pub.publish(ack_msg)
        			
        			brk=True
        			brake_bool.Publish(brk)
        			
        			break
	 
	  #publish brk=False otherwise
	  brake_bool.Publish(brk)
        			
          return
  
def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
