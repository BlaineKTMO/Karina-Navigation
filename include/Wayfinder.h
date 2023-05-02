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

private:
    void reverse();
    void wayfind();

    void reverseCallback(const std_msgs::BoolConstPtr& msg);
    void vfhWptCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void lfWptCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void vfhStatusCallback(const std_msgs::BoolConstPtr& msg);
    void lfStatusCallback(const std_msgs::BoolConstPtr& msg);
    void stopSubCallback(const std_msgs::BoolConstPtr& msg);
    void targetVelocityCallback(const geometry_msgs::TwistConstPtr& msg);
    geometry_msgs::PoseStamped getFromPath(geometry_msgs::PoseStamped goal);
    geometry_msgs::PoseStamped getCurrentPos();

public:
    Wayfinder() :
        wptPub(n.advertise<geometry_msgs::PoseStamped>("/local_goal", 10)),
        reverseSub(n.subscribe("/reverse", 10, &Wayfinder::reverseCallback, this)),
        vfhWptSub(n.subscribe("/vfh_wpt", 10, &Wayfinder::vfhWptCallback, this)),
        lfWptSub(n.subscribe("/lf_wpt", 10, &Wayfinder::lfWptCallback, this)),
        vfhStatusSub(n.subscribe("/vfh_status", 10, &Wayfinder::vfhStatusCallback, this)),
        lfStatusSub(n.subscribe("/lf_status", 10, &Wayfinder::lfStatusCallback, this)),
        stopSub(n.subscribe(STOP_TOPIC, 10, &Wayfinder::stopSubCallback, this)),
        targetVelocitySub(n.subscribe("/target_vel", 10, &Wayfinder::targetVelocityCallback, this)),
        targetVelocityPub(n.advertise<geometry_msgs::Twist>("/target_velocity", 10)),
        velPub(n.advertise<geometry_msgs::Twist>("/move_base/cmd_vel", 10)),
        path_client(n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan")),
        rate(ros::Rate(10)),
        reverseStatus(false),
        vfhStatus(false),
        lfStatus(false),
        stopStatus(false),
        tfListener(tfBuffer)
         {};

    void navigate();

};

#endif
