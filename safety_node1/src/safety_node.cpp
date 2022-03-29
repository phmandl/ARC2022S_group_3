#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fstream>
//#include "safety_node1/precompute.hpp"
#include <ros/console.h>
#include <math.h>
#include <cmath>
#include <vector>


//using namespace racecar_simulator;

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    ros::NodeHandle n_internal;
    double speed;
    // TODO: create ROS subscribers and publishers
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    
    ros::Publisher brake_bool_pub;
    ros::Publisher brake_pub;
    
    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // for collision detection
    double brake_ttc_threshold;
    
    // boolean data variable
    std_msgs:: Bool brk;
    
    static constexpr double PI = 3.1415;
       

public:
    Safety() {
        n = ros::NodeHandle();
        n_internal=ros::NodeHandle("~");
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        
        //ref: behavior_controller.cpp       
        // Make a publisher for brake messages
        brake_bool_pub = n.advertise<std_msgs::Bool>("brake_bool", 1);
        brake_pub= n.advertise<ackermann_msgs::AckermannDriveStamped>("brake",1);
        
        // Start subscribers to listen to laser scan and odom messages
        laser_sub = n.subscribe("scan", 1, &Safety::scan_callback,this);
        odom_sub = n.subscribe("odom", 1, &Safety::odom_callback,this);
               
         // Get params for precomputation and collision detection
        int scan_beams;
        double scan_fov, scan_ang_incr,wheelbase, width,scan_distance_to_base_link;
        //n_internal.getParam("brake_ttc_threshold", brake_ttc_threshold);
        brake_ttc_threshold=.2;
        n_internal.getParam("scan_beams", scan_beams);
        n_internal.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
        n_internal.getParam("width", width);
        n_internal.getParam("wheelbase", wheelbase);
        n_internal.getParam("scan_field_of_view", scan_fov);
        scan_ang_incr = scan_fov / scan_beams;
        
        
        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = get_cosines(scan_beams, -scan_fov/2.0, scan_ang_incr);
        car_distances = get_car_distances(scan_beams, wheelbase, width, 
        scan_distance_to_base_link, -scan_fov/2.0, scan_ang_incr);
     }
    
    void odom_callback(const nav_msgs::Odometry &odom_msg) 
    {
        // TODO: update current speed
        speed = odom_msg.twist.twist.linear.x;
     }

    void scan_callback(const sensor_msgs::LaserScan &scan_msg) 
    {
        brk.data=false;
        
        // TODO: calculate TTC
       //ref: collision detection in behavior_controller.cpp
       
        if(speed!=0)
        {
        	for(size_t i=0;i<scan_msg.ranges.size();i++)
		{
			double proj_speed=speed * cosines[i];
			double ttc=(scan_msg.ranges[i]-car_distances[i])/proj_speed;
			
			if((ttc<brake_ttc_threshold) && (ttc>=0.0))
				{       
				        // TODO: publish drive/brake message
					//ref: mux.cpp
					// Make and publish velocity=0 message 
					ackermann_msgs::AckermannDriveStamped drive_st_msg;
					ackermann_msgs::AckermannDrive drive_msg;
					std_msgs::Header header;
					drive_msg.speed = 0.0;
					drive_msg.steering_angle = 0.0;
					header.stamp = ros::Time::now();
					drive_st_msg.header = header;
					// set drive message in drive stamped message
					drive_st_msg.drive = drive_msg;
					// publish AckermannDriveStamped message to drive topic
					brake_pub.publish(drive_st_msg);
					
					//publish brake true for each scan
					brk.data=true;
		       		brake_bool_pub.publish(brk);
		       		break;
		            	}
		}
	 }
        else 
        {
    	//otherwise publish brake {false} for each scan
	      brake_bool_pub.publish(brk);
	 }	
       
   }
   
   //Aux functions for cosines of scan angles and car distances(from precompute.cpp)
   
   std::vector<double> get_cosines(int scan_beams, double angle_min, double scan_ang_incr) 
   {
    // Precompute distance from lidar to edge of car for each beam
    std::vector<double> cosines = std::vector<double>();
    cosines.reserve(scan_beams);

    for (int i = 0; i < scan_beams; i++) 
    {
        double angle = angle_min + i * scan_ang_incr;
        cosines[i] = std::cos(angle);
    }

    return cosines;
   }
   
   
   std::vector<double> get_car_distances(int scan_beams, double wheelbase, double width, 
    double scan_distance_to_base_link, double angle_min, double scan_ang_incr) 
    {
    // Precompute distance from lidar to edge of car for each beam

    std::vector<double> car_distances = std::vector<double>();
    car_distances.reserve(scan_beams);
    double dist_to_sides = width / 2.0;
    double dist_to_front = wheelbase - scan_distance_to_base_link;
    double dist_to_back = scan_distance_to_base_link;
    // loop through each angle
    for (int i = 0; i < scan_beams; i++) {
        double angle = angle_min + i * scan_ang_incr;

        if (angle > 0) {
            if (angle < PI / 2.0) {
                // between 0 and pi/2
                double to_side = dist_to_sides / std::sin(angle);
                double to_front = dist_to_front / std::cos(angle);
                car_distances[i] = std::min(to_side, to_front);
            } else {
                // between pi/2 and pi
                double to_side = dist_to_sides / std::cos(angle - PI / 2.0);
                double to_back = dist_to_back / std::sin(angle - PI / 2.0);
                car_distances[i] = std::min(to_side, to_back);
            } 
        } else {
            if (angle > -PI / 2.0) {
                // between 0 and -pi/2
                double to_side = dist_to_sides / std::sin(-angle);
                double to_front = dist_to_front / std::cos(-angle);
                car_distances[i] = std::min(to_side, to_front);
            } else {
                // between -pi/2 and -pi
                double to_side = dist_to_sides / std::cos(-angle - PI / 2.0);
                double to_back = dist_to_back / std::sin(-angle - PI / 2.0);
                car_distances[i] = std::min(to_side, to_back);
            }
          }
        }

    return car_distances;
    }



};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}
