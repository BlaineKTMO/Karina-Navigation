#ifndef WAYPOINT_SERVER_H
#define WAYPOINT_SERVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <vector>

const static std::string GOAL_TOPIC = "/goal";
const static std::string WAYPOINT_STATUS_TOPIC = "/wayfindStatus";

class WaypointServer {
protected: 
    ros::NodeHandle n;
    ros::Publisher goal_pub;
    ros::Subscriber waypointStatusSub;
    ros::Rate rate;

private:
    std::vector<geometry_msgs::Point> wpt_vec;
    int counter;
    int goal_count;
    bool wayfind_complete;
    bool wayfind_status;

    void publishGoal(geometry_msgs::PoseStamped goal);
    void wayfind_callback(const std_msgs::BoolConstPtr& msg);
    void advance();
    void recede();

public:
    WaypointServer() :
        goal_pub(n.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 10)),
        waypointStatusSub(n.subscribe(WAYPOINT_STATUS_TOPIC, 10, &WaypointServer::wayfind_callback, this)),
        rate(ros::Rate(1)),
        counter(0),
    wayfind_complete(false)
    { }

    void readGoals(std::string fileName);
    void start();
};

#endif
