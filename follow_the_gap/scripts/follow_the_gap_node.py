#!/usr/bin/env python

import sys
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from dynamic_reconfigure.server import Server
# from wall_follow.cfg import dyn_reconfigConfig

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
NR_SCANS = 1081
ANGLE_INCREMENT = 0.004365153145045042

mid_ray_int = int(NR_SCANS/2)
ind_weght_zero = 150
ind_weght_zero = 110

# creating weighting vec
weighting_vec = np.zeros(NR_SCANS)
for i in range(NR_SCANS):
    weighting_vec[i] = 3 - np.abs(i-mid_ray_int)/mid_ray_int*2
weighting_vec[0:ind_weght_zero] = 0
weighting_vec[NR_SCANS-ind_weght_zero:-1] = 0


CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
HZ = 100
TS = 1.0/HZ

MAX_VELOCITY = 1.0
# MAX_LOOKAHEAD = 1.0
GAP_TH = 2.4*1.1
GAP_TH = 4
RED_GAP = 8 #Rays
Gap_Clearance = 0.4 # 30 cm --> Disperity

class GapFollow:
    """ Implement Follow the Gap   """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        gap_topic = '/scan_gap'
        drive_topic = '/nav'

        message = LaserScan()
        self.laserdata = np.ones(NR_SCANS)
        self.laser_fullsmessage = message
        self.midRayRange = 0
        print(message)

        # self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)
        self.gap_pub = rospy.Publisher(gap_topic, LaserScan)

        # dynamic reconfigure server
        #self.srv = Server(dyn_reconfigConfig, self.dynCallback)
        # rosrate
        self.r = rospy.Rate(HZ)
        # PID Stuff
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.01
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

    def get_gaps(self,thres,ranges_orig):

        ranges = np.ones(len(weighting_vec))*0.4
        ranges[weighting_vec>0] = ranges_orig[weighting_vec>0]

        ind_gap = np.zeros(len(ranges))
        ind_gap[ranges>thres]=1;
        # print(np.sum(ranges))
        start_ind = np.array([]).astype(int)
        end_ind = np.array([]).astype(int)
        sum_temp=0
        for i in range(len(ranges)):
            if ind_gap[i]==1:
                if sum_temp==0:
                    start_ind = np.append(start_ind,i)
                sum_temp = sum_temp+1
            else:
                if sum_temp>0:
                    end_ind = np.append(end_ind,i)
                sum_temp = 0
        if sum_temp>0:
            end_ind = np.append(end_ind,i)

        # Clear small gaps --> check if gap is big enough
        length_of_gap = end_ind - start_ind
        #print('Length of Gap: ',length_of_gap)
        start_ind = start_ind[length_of_gap>2*RED_GAP].astype(int)
        end_ind = end_ind[length_of_gap>2*RED_GAP].astype(int)


        if len(start_ind)>0: #disperity
            # Change first or last index --> for dispersity
            if start_ind[0]==0:
                start_ind[0]=1
            if end_ind[-1]==len(ranges)-1:
                end_ind[-1]=len(ranges)-2

            dist_start = ranges[start_ind-1]
            dist_end = ranges[end_ind+1]
            red_start = (Gap_Clearance/dist_start/ANGLE_INCREMENT).astype(int)
            red_end = (Gap_Clearance/dist_end/ANGLE_INCREMENT).astype(int)

            start_ind = start_ind+red_start
            end_ind = end_ind-red_end

            # Check if Gap is big enough
            length_of_gap = end_ind - start_ind
            start_ind = start_ind[length_of_gap>2*RED_GAP].astype(int)
            end_ind = end_ind[length_of_gap>2*RED_GAP].astype(int)

        else:
            red_start = []
            red_end = []
            dist_start = []
            dist_end = []
        return start_ind,end_ind,red_start,red_end,dist_start,dist_end

    def calc_Gap(self):
        new_gap_th = GAP_TH
        ranges = np.array(self.laserdata)
        # calvulate gaps
        #start_ind,end_ind,red_start,red_end,dist_start,dist_end = self.get_gaps(new_gap_th,ranges)
        #print('start_ind: ', start_ind,'end_ind: ',end_ind)

        i = 0
        start_ind = []
        while len(start_ind)<1:
            # recompute with smaller threshold
            new_gap_th = GAP_TH-0.4*i
            start_ind,end_ind,red_start,red_end,dist_start,dist_end = self.get_gaps(new_gap_th,ranges)
            #print('Recomputed ...i = ',i,'Thres new = ',new_gap_th)
            i = i+1
            if (i+1)*0.4>GAP_TH:
                break
        # select gap
        weight = np.zeros(len(start_ind))
        gap_mean = np.zeros(len(start_ind))
        for i in range(len(start_ind)):
            weight[i] = np.sum(ranges[start_ind[i]:end_ind[i]])
            gap_mean[i] = np.mean(ranges[start_ind[i]:end_ind[i]])
        if len(start_ind)>0:
            ind_max = np.argmax(weight)
            gap_weigth = weight[ind_max]
            gap_mean = gap_mean[ind_max]
            if 1:
                summe = 0
                for i in range(end_ind[ind_max] - start_ind[ind_max]):
                    summe = summe + ranges[start_ind[ind_max]+i]*i
                Laser_ind_to_head = start_ind[ind_max] + summe/weight[ind_max]
                #Laser_ind_to_head = (end_ind[ind_max] + start_ind[ind_max])/2
            else:
                Laser_ind_to_head = (end_ind[ind_max] + start_ind[ind_max])/2
            self.delta = Laser_ind_to_head-NR_SCANS/2
        else:
            ind_max = np.array([]).astype(int)
            gap_weigth = 0
            gap_mean = 0
            self.delta = 0

        self.gap_weight = gap_weigth
        self.gap_mean = gap_mean

        # Create the message to vizulize in RVIZ
        gap_msg_ranges = ranges
        gap_msg_intensity = ranges/1000+1

        # print(red_end)
        for i in range(len(start_ind)):
            # dist start
            #print(start_ind)
            #print(ind_max)
            gap_msg_intensity[start_ind[i]:end_ind[i]] = 10
            gap_msg_ranges[start_ind[i]:end_ind[i]]=new_gap_th

            gap_msg_intensity[start_ind[i]-red_start[i]:start_ind[i]] = 20
            gap_msg_ranges[start_ind[i]-red_start[i]:start_ind[i]] = dist_start[i]
            gap_msg_intensity[end_ind[i]:end_ind[i]+red_end[i]] = 20
            gap_msg_ranges[end_ind[i]:end_ind[i]+red_end[i]] = dist_end[i]
        # selected gap
        if len(start_ind)>0:
            gap_msg_intensity[start_ind[ind_max]:end_ind[ind_max]]=40
            gap_msg_ranges[start_ind[ind_max]:end_ind[ind_max]]=new_gap_th
            # heading direction
            gap_msg_intensity[int(Laser_ind_to_head)-4:int(Laser_ind_to_head)+4] = 30

        self.gap_msg_ranges = gap_msg_ranges
        self.gap_msg_intensity = gap_msg_intensity

    def followGap(self):
        # Calculate Gap
        self.calc_Gap()
        # calculate error
        self.e0 = -self.delta*0.002*1.1*1.1
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

        # Lidar message
        gap_msg = LaserScan
        #gap_msg = self.laser_fullsmessage
        #gap_msg.header.stamp = rospy.Time.now()
        #gap_msg.header.frame_id = "laser"
        gap_msg = self.laser_fullsmessage
        gap_msg.ranges = self.gap_msg_ranges
        gap_msg.intensities = self.gap_msg_intensity
        self.gap_pub.publish(gap_msg)

    def lidar_callback(self, data):
        self.laserdata = data.ranges
        self.laser_fullsmessage = data
        self.midRayRange = data.ranges[mid_ray_int]

    def calcVelocity(self):
        abs_deg_angle = np.abs(np.rad2deg(self.angle))
        #v = 3+self.midRayRange*0.4+self.gap_mean*0.08 #- abs_deg_angle*0.005
        #self.velocity = v*0.5*0.8
        v = 0.8+self.midRayRange*0.3+self.gap_weight*0.0015 #- abs_deg_angle*0.005
        self.velocity = np.min((v,MAX_VELOCITY))
        # print(v)

def main(args):
    rospy.init_node("follow_the_gap", anonymous=True)
    gf = GapFollow()
    while not rospy.is_shutdown():
        gf.followGap()
        gf.r.sleep()

if __name__=='__main__':
	main(sys.argv)
