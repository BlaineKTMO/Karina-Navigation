/**
* Author: Blaine Oania
* File: Wayfinder.h
* Date: 3/31/2023
* Description:
*   Subsumptive navigation planner that selects an incoming waypoint from input
*   topics and publishes chosen point to be read by a local controller. If no
*   waypoint is available, a waypoint in front of the robot is chosen.
*
* Input Topics
*   vfhWpt: Vector Field Histogram waypoint
*   lfWpt: Lane Follow waypoint
*   targetVel: Target Velocity 
*   stop: Stop signal
*   reverse: Reverse signal
*
* Output Topics
*   localGoal: Chosen local waypoint
*   targetVelocity: Chosen target velocity
*/

#ifndef WAYFINDER_H
#define WAYFINDER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

const std::string STOP_TOPIC = "/stop";

class Wayfinder {
protected:
    ros::NodeHandle n;
    ros::Rate rate;

    ros::Publisher wptPub;
    ros::Publisher targetVelocityPub;
    ros::Publisher velPub; 

    ros::Subscriber reverseSub;
    ros::Subscriber vfhWptSub;
    ros::Subscriber lfWptSub;
    ros::Subscriber vfhStatusSub;
    ros::Subscriber lfStatusSub;
    ros::Subscriber stopSub;
    ros::Subscriber targetVelocitySub;

    ros::ServiceClient path_client;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool reverseStatus;
    bool vfhStatus;
    bool lfStatus;
    bool stopStatus;

    geometry_msgs::PoseStamped vfhWpt;
    geometry_msgs::PoseStamped lfWpt;

    geometry_msgs::PoseStamped wpt;
    geometry_msgs::Twist targetVelocity;

    std::vector<geometry_msgs::PoseStamped> collectedPath;
    nav_msgs::Path path;

private:
    // Navigation functions
    void reverse();
    void wayfind();

    // Agent subscribers
    void reverseCallback(const std_msgs::BoolConstPtr& msg);
    void stopSubCallback(const std_msgs::BoolConstPtr& msg);
    void targetVelocityCallback(const geometry_msgs::TwistConstPtr& msg);

    // Waypoint subscribers
    void vfhWptCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void lfWptCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void vfhStatusCallback(const std_msgs::BoolConstPtr& msg);
    void lfStatusCallback(const std_msgs::BoolConstPtr& msg);


    // Helper Functions
    geometry_msgs::PoseStamped getFromPath(geometry_msgs::PoseStamped goal);
    geometry_msgs::PoseStamped getCurrentPos();
    void addToPath(geometry_msgs::PoseStamped goal);
    float getDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

public:
    Wayfinder() :
        // Initialize publishers
        targetVelocityPub(n.advertise<geometry_msgs::Twist>("/target_velocity", 10)),
        wptPub(n.advertise<geometry_msgs::PoseStamped>("/local_goal", 10)),
        velPub(n.advertise<geometry_msgs::Twist>("/move_base/cmd_vel", 10)),

        // Initialize subscribers
        reverseSub(n.subscribe("/reverse", 10, &Wayfinder::reverseCallback, this)),
        vfhWptSub(n.subscribe("/vfh/waypoint", 10, &Wayfinder::vfhWptCallback, this)),
        lfWptSub(n.subscribe("/lf_wpt", 10, &Wayfinder::lfWptCallback, this)),
        vfhStatusSub(n.subscribe("/vfh_status", 10, &Wayfinder::vfhStatusCallback, this)),
        lfStatusSub(n.subscribe("/lf_status", 10, &Wayfinder::lfStatusCallback, this)),
        stopSub(n.subscribe(STOP_TOPIC, 10, &Wayfinder::stopSubCallback, this)),
        targetVelocitySub(n.subscribe("/target_vel", 10, &Wayfinder::targetVelocityCallback, this)),

        // Initialize move base path service
        path_client(n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan")),

        // Set loop rate
        rate(ros::Rate(10)),
        
        // Set initial status values
        reverseStatus(false),
        vfhStatus(false),
        lfStatus(false),
        stopStatus(false),

        // Initialize transform listener
        tfListener(tfBuffer),
        collectedPath({})
         {};

    void navigate();

};

#endif
