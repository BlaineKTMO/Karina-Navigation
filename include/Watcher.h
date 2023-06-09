/**
* Author: Blaine Oania
* File: Watcher.h
* Date: 3/31/2023
* Description:
*   Watchdog that monitors robot sensor and costmap topics to prevent collisions
*   and trigger recovery behaviors.
*
* Inputs:
*   lidar: lidar sensor topic
*   costmap: costmap topic
*
* Outputs:
*   targetVel: Chosen target velocity
*   stop: Stop signal
*   reverse: Reverse signal
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h> 
#include <sensor_msgs/LaserScan.h>

const std::string LIDAR_TOPIC = "/scan";
const std::string STOP_TOPIC = "/stop";
const std::string REVERSE_TOPIC = "/reverse";

static const float MIN_DISTANCE = 2;

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
    ros::Publisher targetVelPub;
    ros::Publisher stopPub;
    ros::Publisher reversePub;
    
    geometry_msgs::Twist targetVel;
    std_msgs::Bool falseStdMsg;
    std_msgs::Bool trueStdMsg;

    void checkLaserCollision(const sensor_msgs::LaserScanConstPtr& msg);
    // void checkCostmapCollision();

    ros::Rate rate;


public:
    Watcher() : 
        lidarSub(n.subscribe(LIDAR_TOPIC, 10, &Watcher::checkLaserCollision, this)),
        // cameraSub(n.subscribe(CAMERA_TOPIC, 10, &Watcher::))

        // vfhStatPub(n.advertise<std_msgs::Bool>("/vfh_status", 10)),
        // lfStatPub(n.advertise<std_msgs::Bool>("/lf_status", 10)),
        targetVelPub(n.advertise<geometry_msgs::Twist>("/target_vel", 10)),
        stopPub(n.advertise<std_msgs::Bool>(STOP_TOPIC, 10)),
        reversePub(n.advertise<std_msgs::Bool>(REVERSE_TOPIC, 10)),
        rate(ros::Rate(5))
    {}

    void monitor(); 
};
