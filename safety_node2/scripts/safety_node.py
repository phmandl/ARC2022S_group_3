#!/usr/bin/env python3
import rospy

# TODO: import ROS msg types and libraries

import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
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
        rospy.Subscriber("/odom",Odometry,self.odom_callback)
        rospy.Subscriber("/scan",LaserScan,self.scan_callback)
        
        self.brake_bool_pub=rospy.Publisher("brake_bool",Bool,queue_size=1)
        self.brake_pub=rospy.Publisher("brake",AckermannDriveStamped,queue_size=1)
        
        #self.brake_ttc_threshold=rospy.get_param('~brake_ttc_threshold')
        self.brake_ttc_threshold=.3
        scan_beams=rospy.get_param('~scan_beams')
        wheelbase=rospy.get_param('~wheelbase')
        width=rospy.get_param('~width')
        scan_dist_to_base_link=rospy.get_param('~scan_distance_to_base_link')
        scan_fov=rospy.get_param('~scan_field_of_view')
        scan_angle_incr=scan_fov/scan_beams
        
        self.cos,self.car_dist=self.cos_car_dist(scan_beams, 
		                                 wheelbase, 
		                                 width, 
		                                 scan_dist_to_base_link,
		                                 -scan_fov/2.0,
		                                 scan_angle_incr)
	       

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
                      
    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        
        if(self.speed!=0):	   
            proj_speed=self.speed*self.cos
            ttc=np.amin((scan_msg.ranges-self.car_dist)/np.maximum(proj_speed,0.00001))
            	                                             		
            # TODO: publish brake message and publish controller bool
        		
            if(ttc<self.brake_ttc_threshold and ttc>=0):
                    ack_msg=AckermannDriveStamped()
                    ack_msg.header.stamp=rospy.Time.now()
                    ack_msg.drive.steering_angle=0.0
                    ack_msg.drive.speed=0.0
                    self.brake_pub.publish(ack_msg)
                                                  
                    brk=True
                    self.brake_bool_pub.publish(brk)
		
                    return 
	 
        #publish brk=False otherwise
        else:
        	brk=False
        	self.brake_bool_pub.publish(brk)
        			
        return
        
    def cos_car_dist(self,scan_beams,wheelbase,width,scan_distance_to_base_link,angle_min,angle_incr):
                   
        car_distances=np.empty(scan_beams)
        cosines=np.empty(scan_beams)	
        dist_to_sides=width/2.0
        dist_to_front=wheelbase-scan_distance_to_base_link
        dist_to_back=scan_distance_to_base_link

        angle=[angle_min+i*angle_incr for i in range(scan_beams)]
        cosines=np.cos(angle)
        c1=[min(dist_to_sides/np.sin(angle[i]),dist_to_front/np.cos(angle[i]))
                          for i in range(int((0-angle_min)/angle_incr),int((np.pi/2-angle_min)/angle_incr))]
        c2=[min(dist_to_sides/np.cos(angle[i]-np.pi/2),dist_to_back/np.sin(angle[i]-np.pi/2))
                          for i in range(int((np.pi/2-angle_min)/angle_incr),len(angle))]
        c3=[min(dist_to_sides/np.sin(-angle[i]),dist_to_front/np.cos(-angle[i]))
                          for i in range(int((-np.pi/2-angle_min)/angle_incr),int((0-angle_min)/angle_incr))]
        c4=[min(dist_to_sides/np.cos(-angle[i]-np.pi/2),dist_to_back/np.sin(-angle[i]-np.pi/2))
                          for i in range(int((-np.pi/2-angle_min)/angle_incr))]
        car_distances=np.append(np.append(np.append(c4,c3),c1),c2)    
        return cosines,car_distances      	         	 
  
def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
