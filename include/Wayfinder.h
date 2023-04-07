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
    ros::Subscriber vfhWptSub;
    ros::Subscriber lfWptSub;
    ros::Subscriber vfhStatusSub;
    ros::Subscriber lfStatusSub;
    ros::Subscriber stopSub;
    ros::Subscriber targetVelocitySub;
    ros::Publisher targetVelocityPub;
    ros::ServiceClient path_client;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool vfhStatus;
    bool lfStatus;
    bool stopStatus;

    geometry_msgs::PoseStamped vfhWpt;
    geometry_msgs::PoseStamped lfWpt;

    geometry_msgs::PoseStamped wpt;
    geometry_msgs::Twist targetVelocity;

private:
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
        vfhWptSub(n.subscribe("/vfh_wpt", 10, &Wayfinder::vfhWptCallback, this)),
        lfWptSub(n.subscribe("/lf_wpt", 10, &Wayfinder::lfWptCallback, this)),
        vfhStatusSub(n.subscribe("/vfh_status", 10, &Wayfinder::vfhStatusCallback, this)),
        lfStatusSub(n.subscribe("/lf_status", 10, &Wayfinder::lfStatusCallback, this)),
        stopSub(n.subscribe(STOP_TOPIC, 10, &Wayfinder::stopSubCallback, this)),
        targetVelocitySub(n.subscribe("/target_vel", 10, &Wayfinder::targetVelocityCallback, this)),
        targetVelocityPub(n.advertise<geometry_msgs::Twist>("/target_velocity", 10)),
        path_client(n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan")),
        rate(ros::Rate(10)),
        vfhStatus(false),
        lfStatus(false),
        stopStatus(false),
        tfListener(tfBuffer)
         {};

    void wayfind();
};

#endif
