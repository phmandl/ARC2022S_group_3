// #include <ros/ros.h>
// #include <iostream>
// #include "std_msgs/String.h"
// #include <sstream>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/LaserScan.h>
// #include <ackermann_msgs/AckermannDriveStamped.h>
// // TODO: include ROS msg type headers and libraries

// class Safety {
// // The class that handles emergency braking
// private:
//     ros::NodeHandle n;
//     double speed;
//     // TODO: create ROS subscribers and publishers

//     // Odometry Subscriber
//     ros::Subscriber odom_sub;
//     // LiDar Subscriber
//     ros::Subscriber laser_sub;

//     // Break Publisher
//     ros::Publisher break_bool_pub;
//     ros::Publisher break_pub;

//     // Calculation to compensate the position of the laser scan
//     // Get params for precomputation and collision detection
//     int scan_beams;                     // number of beams 1080
//     double scan_field_of_view;          //
//     double scan_ang_incr;
//     double wheelbase;
//     double width;
//     double scan_distance_to_base_link;
//     double brake_ttc_threshold;         // for collision detection


// public:
//     Safety() {

//         // Initilization Params
//         n = ros::NodeHandle();
//         speed = 0.0;
        
//         n.getParam("brake_ttc_threshold", brake_ttc_threshold);
//         n.getParam("scan_beams", scan_beams);
//         n.getParam("scan_distance_to_base_link", scan_distance_to_base_link);
//         n.getParam("width", width);
//         n.getParam("wheelbase", wheelbase);
//         n.getParam("scan_field_of_view", scan_field_of_view);
//         scan_ang_incr = scan_field_of_view / scan_beams;
//         /*
//         One publisher should publish to the /brake topic with an
//         ackermann_msgs/AckermannDriveStamped brake message.

//         One publisher should publish to the /brake_bool topic with a
//         std_msgs/Bool message.

//         You should also subscribe to the /scan topic to get the
//         sensor_msgs/LaserScan messages and the /odom topic to get
//         the nav_msgs/Odometry messages

//         The subscribers should use the provided odom_callback and 
//         scan_callback as callback methods

//         NOTE that the x component of the linear velocity in odom is the speed
//         */

//         // TODO: create ROS subscribers and publishers

//         // Start subscribers to listen to laser scan and odom messages
//         laser_sub = n.subscribe("scan",1,&Safety::scan_callback,this);
//         odom_sub = n.subscribe("odom",1,&Safety::odom_callback,this);

//         // Make a publisher for brake messages
//         break_bool_pub = n.advertise<std_msgs>("brake_bool",1);
//         break_pub = n.advertise<ackerm
//     }
//     void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
//         // TODO: update current speed
//         double x_velocity = odom_msg->twist.twist.linear.x;
//         speed = x_velocity;
//     }

//     void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        
//         // get Laser-scan Infos for 
//         std::vector<float> ->ranges
//         // TODO: calculate TTC


//         // TODO: publish drive/brake message
//     }

// };

// int main(int argc, char ** argv){
//      //Initialize and start the node
//      ros::init(argc, argv, "safety_node");
//      ros::NodeHandle n;
//      ROS_INFO_STREAM("starting with safety brake") ;
//     std::cout << "safety_node node initialised" << std::endl;
//     //handle ROS communication events
//     /* The second parameter to advertise() is the size of the message queue
//       * used for publishing messages.  If messages are published more quickly
//       * than we can send them, the number here specifies how many messages to
//       * buffer up before throwing some away.
//         */
//     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);  
//     ros::Rate loop_rate(10);
//     int count=0;

//     while (ros::ok())
//     {
//         std_msgs::String msg;

//         std::stringstream ss;
//         ss << "hello" << count;
//         msg.data=ss.str();

//         ROS_INFO("%s", msg.data.c_str());

//         chatter_pub.publish(msg);
//         ros::spinOnce();
//         loop_rate.sleep();
//         count++;

//     }
    
//     return 0;
// }
