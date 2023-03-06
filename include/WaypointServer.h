#ifndef WAYPOINT_SERVER_H
#define WAYPOINT_SERVER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <fstream>
#include <vector>

const static std::string GOAL_TOPIC = "/goal";

class WaypointServer {
protected: 
    ros::NodeHandle n;
    ros::Publisher goal_pub;
    ros::Rate rate;

private:
    std::vector<geometry_msgs::Point> wpt_vec;
    int counter;
    int goal_count;

    void publishGoal(geometry_msgs::PoseStamped goal);
    void readGoals(std::string fileName);
    void advance();
    void recede();

    void start();

public:
    WaypointServer() :
        goal_pub(n.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 10)),
        rate(ros::Rate(0.25)),
        counter(0)
    { }
};

#endif
