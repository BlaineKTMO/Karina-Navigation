#ifndef WAYFINDER_H
#define WAYFINDER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <vector>

class Wayfinder {
protected:
    ros::NodeHandle n;
    ros::Rate rate;

    ros::Publisher wptPub;
    ros::Subscriber vfhWptSub;
    ros::Subscriber lfWptSub;
    ros::Subscriber vfhStatusSub;
    ros::Subscriber lfStatusSub;
    ros::Subscriber targetVelocitySub;
    ros::Publisher targetVelocityPub;

    bool vfhStatus;
    bool lfStatus;

    geometry_msgs::PoseStamped vfhWpt;
    geometry_msgs::PoseStamped lfWpt;

    geometry_msgs::PoseStamped wpt;
    geometry_msgs::Twist targetVelocity;

private:
    void vfhWptCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void lfWptCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void vfhStatusCallback(const std_msgs::BoolConstPtr& msg);
    void lfStatusCallback(const std_msgs::BoolConstPtr& msg);
    void targetVelocityCallback(const geometry_msgs::TwistConstPtr& msg);
    geometry_msgs::Point getCurrentPos();

public:
    Wayfinder() :
        wptPub(n.advertise<geometry_msgs::PoseStamped>("/local_goal", 10)),
        vfhWptSub(n.subscribe("/vfh_wpt", 10, &Wayfinder::vfhWptCallback, this)),
        lfWptSub(n.subscribe("/lf_wpt", 10, &Wayfinder::lfWptCallback, this)),
        vfhStatusSub(n.subscribe("/vfh_status", 10, &Wayfinder::vfhStatusCallback, this)),
        lfStatusSub(n.subscribe("/lf_status", 10, &Wayfinder::lfStatusCallback, this)),
        targetVelocitySub(n.subscribe("/target_vel", 10, &Wayfinder::targetVelocityCallback, this)),
        targetVelocityPub(n.advertise<geometry_msgs::Twist>("/target_velocity", 10)),
        rate(ros::Rate(20)),
        vfhStatus(false),
        lfStatus(false)
    {};

    void wayfind();
};

#endif
