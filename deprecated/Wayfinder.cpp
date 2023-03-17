#include "Wayfinder.h"

void Wayfinder::goal_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
    ROS_INFO("Goal callback initiated . . .");
    geometry_msgs::PoseStamped goal;

    // goal.pose = msg->pose;
    // goal.header = msg->header;
    
    goal = *msg.get();

    // wayfind(goal);
    geometry_msgs::PoseStamped initial;
    geometry_msgs::PoseStamped localGoal;

    initial.pose.position = getCurrentPos();
    initial.header.frame_id = "map";

    localGoal = goal;
    ROS_INFO("Starting to move to goal . . .");
    while (distanceToWpt(initial, localGoal) > 1)
    {
        ROS_INFO("Sleeping until local goal reached . . .");
        publishLocalGoal(localGoal);
        initial.pose.position = getCurrentPos();
        rate.sleep();
    }
    
    ROS_INFO("Reached goal");
    std_msgs::Bool ret;

    ret.data = true;

    statusPub.publish(ret);
    statusPub.publish(ret);
    statusPub.publish(ret);
    statusPub.publish(ret);
}

void Wayfinder::wayfind(geometry_msgs::PoseStamped goal) {
    geometry_msgs::PoseStamped initial;
    geometry_msgs::PoseStamped localGoal;

    initial.pose.position = getCurrentPos();
    initial.header.frame_id = "map";

    localGoal = goal;
    ROS_INFO("Starting to move to goal . . .");
    while (distanceToWpt(initial, localGoal) > 1)
    {
        ROS_INFO("Sleeping until local goal reached . . .");
        publishLocalGoal(localGoal);
        initial.pose.position = getCurrentPos();
        rate.sleep();
    }

    std_msgs::Bool ret;
    ret.data = true;
    statusPub.publish(ret);
    statusPub.publish(ret);
    statusPub.publish(ret);

}

void Wayfinder::publishLocalGoal(geometry_msgs::PoseStamped localGoal) {
    localGoalPub.publish(localGoal); 
}

/**
* @brief Calculates distance between two positions
*
* @param initial_pos first position
* @param goal second position
* @return distance
*/
float Wayfinder::distanceToWpt(geometry_msgs::PoseStamped initial_pos, geometry_msgs::PoseStamped goal)
{
    float dx = goal.pose.position.x - initial_pos.pose.position.x;
    float dy = goal.pose.position.y - initial_pos.pose.position.y;

    return pow((dx * dx) + (dy * dy), 0.5);
}

/**
* @brief Returns the current odom position
*
* @return geometry_msgs::Point
*/
geometry_msgs::Point Wayfinder::getCurrentPos()
{

    nav_msgs::OdometryConstPtr odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    geometry_msgs::Point ret;
    if (odom_ptr == NULL)
        ROS_INFO("No odom data found.");
    else
        ret = odom_ptr->pose.pose.position; 

    return ret;
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "wayfinder");

    Wayfinder wayfinder;

    ros::spin();

    return 0;
}
