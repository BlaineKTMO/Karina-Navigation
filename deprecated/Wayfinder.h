#ifndef WAYFINDER_H
#define WAYFINDER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

const std::string LOCAL_GOAL_TOPIC = "/local_goal";
const std::string GOAL_TOPIC = "/goal";
const std::string STATUS_TOPIC = "/wayfindStatus";

class Wayfinder {
protected:
    ros::NodeHandle n;
    ros::Publisher localGoalPub;
    ros::Publisher statusPub;
    ros::Subscriber goalSub;
    ros::Rate rate;

private:
    void goal_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    void wayfind(geometry_msgs::PoseStamped goal);
    void publishLocalGoal(geometry_msgs::PoseStamped localGoal);
    float distanceToWpt(geometry_msgs::PoseStamped initial, geometry_msgs::PoseStamped goal);
    geometry_msgs::Point getCurrentPos();
    
public:
    Wayfinder() :
        localGoalPub(n.advertise<geometry_msgs::PoseStamped>(LOCAL_GOAL_TOPIC, 10)),
        statusPub(n.advertise<std_msgs::Bool>(STATUS_TOPIC, 10)),
        goalSub(n.subscribe(GOAL_TOPIC, 10, &Wayfinder::goal_callback, this)),
        rate(ros::Rate(1))
    {}
};

#endif 
