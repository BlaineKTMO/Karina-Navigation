/**
* Author: Blaine Oania
* File: Watcher
* Date: 3/31/2023
* Description:
*   Watcher node to monitor robot position and sensors.
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h> 
#include <sensor_msgs/LaserScan.h>

const std::string LIDAR_TOPIC = "/scan";
const std::string STOP_TOPIC = "/stop";

static const float MIN_DISTANCE = 1;

class Watcher {
private:
    ros::NodeHandle n;
    
    // Lidar
    // Camera
    // Costmap
    // Odometry
    // Target goal
    
    ros::Subscriber lidarSub; 
    // ros::Subscriber cameraSub;
    // ros::Subscriber costmapSub;
    // ros::Subscriber odomSub;
    // ros::Subscriber targetSub;

    // ros::Subscriber lane
    
    // ros::Publisher vfhStatPub;
    // ros::Publisher lfStatPub;
    // ros::Publisher targetVelPub;
    ros::Publisher stopPub;
    
    bool stop;

    void checkLaserCollision(const sensor_msgs::LaserScanConstPtr& msg);
    // void checkCostmapCollision();

    ros::Rate rate;



public:
    Watcher() : 
        lidarSub(n.subscribe(LIDAR_TOPIC, 10, &Watcher::checkLaserCollision, this)),
        // cameraSub(n.subscribe(CAMERA_TOPIC, 10, &Watcher::))

        // vfhStatPub(n.advertise<std_msgs::Bool>("/vfh_status", 10)),
        // lfStatPub(n.advertise<std_msgs::Bool>("/lf_status", 10)),
        // targetVelPub(n.advertise<geometry_msgs::Twist>("/target_vel", 10)),
        stopPub(n.advertise<std_msgs::Bool>(STOP_TOPIC, 10)),
        stop(false),
        rate(ros::Rate(5))
    {}

    void monitor(); 
};
