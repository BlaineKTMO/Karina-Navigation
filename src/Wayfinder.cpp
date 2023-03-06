#ifndef WAYFINDER_H
#define WAYFINDER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

const std::string LOCAL_GOAL_TOPIC = "/local_goal";
const std::string GOAL_TOPIC = "/goal";

class Wayfinder {
protected:
    ros::NodeHandle n;
    ros::Publisher local_goal;
    ros::Subscriber goal_sub;
    ros::Rate rate;
private:
    void goal_callback(geometry_msgs::PoseStampedConstPtr& msg);
    void wayfind();
    float distanceToWpt(const geometry_msgs::PoseStampedConstPtr& initial, const geometry_msgs::PoseStampedConstPtr& goal);
    geometry_msgs::PointConstPtr getCurrentPos();
    
public:
    Wayfinder() :
        local_goal(n.advertise<geometry_msgs::PoseStamped>(LOCAL_GOAL_TOPIC, 10)),
        goal_sub(n.subscribe(GOAL_TOPIC, 10, &Wayfinder::goal_callback, this)),
        rate(ros::Rate(1))
    {}
};

#endif 
