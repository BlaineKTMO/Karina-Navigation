#ifndef WAYPOINT_SERVER_H
#define WAYPOINT_SERVER_H
#if defined __cplusplus

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

const static std::string GOAL_TOPIC = "/goal";

class WaypointServer {
protected: 
    ros::NodeHandle n;
    ros::Publisher goal_pub;
private:
    void publishGoal(geometry_msgs::PoseStamped goal);
    void readGoals(std::string fileName);
    void advance();
    void recede();

public:
    WaypointServer() :
        goal_pub(n.advertise<geometry_msgs::PoseStamped>(GOAL_TOPIC, 10))
    { }
};

#endif
#endif
