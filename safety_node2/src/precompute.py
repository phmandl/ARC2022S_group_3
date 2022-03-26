#!usr/bin/env python

import rospy
import numpy as np
import math

class precompute():
	def __init__(self, 
		     scan_beams, 
		     wheelbase, 
		     width, 
		     scan_distance_to_base_link,
		     angle_min,
		     scan_angle_incr):
        self.scan_beams=scan_beams
        self.dist_to_sides=width/2
        self.dist_to_front=wheelbase-scan_distance_to_base_link
        self.dist_to_back=scan_distance_to_base_link
        self.angle_min=angle_min
        self.angle_incr=scan_angle_incr
        
        self.car_distances=np.empty(scan_beams)
        self.cosines=np.empty(scan_beams)
        
        def cosines_and_car_dist(self):
        	for i in range(self.scan_beams):
        	
        		angle=self.angle_min+ i*self.angle_incr
        		self.cosines[i]=np.cos(angle)
        		
        		if (angle>0):
        		       
				 if (angle<np.pi/2):
					
					 to_side=self.dist_to_sides/np.sin(angle)
					 to_front=self.dist_to_front/np.cos(angle)
					 self.car_distances[i]=np.min(to_side,to_front)
									
				 else:
				         to_side=self.dist_to_sides/np.cos(angle-np.pi/2)
				         to_back=self.dist_to_back/np.sin(angle-np.pi/2)
				         self.car_distances[i]=np.min(to_side,to_back)
				         
		         else:
		         	 if (angle>-np.pi/2):
		         	 	
		         	         to_side=self.dist_to_sides/np.sin(-angle)
		         	         to_front=self.dist_to_front/np.cos(-angle)
		         	         self.car_distances[i]=np.min(to_side,to_front)
		         	         
	         	         else:
	         	         	 to_side=self.dist_to_sides/np.cos(-angle-np.pi/2)
	         	         	 to_back=self.dist_to_back/np.sin(-angle-np.pi/2)
	         	         	 self.car_distances[i]=np.min(to_side,to_back)
	         	         	 
	    return self.cosines,self.car_distances      	         	 
		            
				          
        		
        		
        

