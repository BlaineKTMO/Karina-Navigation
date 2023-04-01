#include <ros/ros.h>
#include <Wayfinder.h>

void Wayfinder::vfhWptCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    vfhWpt = *msg.get(); 
}

void Wayfinder::lfWptCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    lfWpt = *msg.get();
}

void Wayfinder::vfhStatusCallback(const std_msgs::BoolConstPtr& msg) {
    vfhStatus = msg->data;
}

void Wayfinder::lfStatusCallback(const std_msgs::BoolConstPtr& msg) {
    lfStatus = msg->data;
}

void Wayfinder::stopSubCallback(const std_msgs::BoolConstPtr& msg) {
    stopStatus = true; 
} 

void Wayfinder::targetVelocityCallback(const geometry_msgs::TwistConstPtr& msg) {
    targetVelocity = *msg.get();
}

void Wayfinder::wayfind() {
    wpt = getCurrentPos();
    wpt.pose.position.x += 1.5;

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 10;
    goal.pose.position.y = 10;
    goal.pose.orientation.w = 1;

    if(vfhStatus) {
        // wpt = vfhWpt;
        // wpt.pose.position.y += 5;
        wpt = getFromPath(goal);
    }
    if(lfStatus) {
        // wpt = lfWpt;
        // wpt.pose.position.y -= 5;
        goal.pose.position.x = -10;
        wpt = getFromPath(goal);
    }
    if(stopStatus) {
        return;
    }
    
    targetVelocityPub.publish(targetVelocity);
    wptPub.publish(wpt);

    rate.sleep();
}

/**
* @brief Returns the current odom position
*
* @return geometry_msgs::Point
*/
geometry_msgs::PoseStamped Wayfinder::getCurrentPos()
{
    nav_msgs::OdometryConstPtr odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
    geometry_msgs::PoseStamped ret;
    if (odom_ptr == NULL)
        ROS_INFO("No odom data found.");
    else
        ret.pose = odom_ptr->pose.pose;

    ret.header.frame_id = "map";
    ret.pose.orientation.x = 0;
    ret.pose.orientation.y = 0;
    ret.pose.orientation.z = 0;
    ret.pose.orientation.w = 1;

    return ret;
}

geometry_msgs::PoseStamped Wayfinder::getFromPath(geometry_msgs::PoseStamped goal) 
{
    nav_msgs::GetPlan path_service;
    path_service.request.goal = goal;
    path_service.request.start = getCurrentPos();
    path_service.request.tolerance = 3;

    geometry_msgs::PoseStamped ret;

    if(path_client.call(path_service))
    {
        ROS_INFO("Path service called sucessfully!");
        if (path_service.response.plan.poses.size() >= 31) {
            ROS_INFO("Sending path at index 30.");
            ret = path_service.response.plan.poses.at(30);
        }
        else {
            ROS_INFO("Sending end of path.");
            ret = *(path_service.response.plan.poses.cend() - 1);
        }
    }

    return ret;
}


int main (int argc, char *argv[])
{
    ros::init(argc, argv, "wayfinder");
    
    Wayfinder wayfinder;
    while(ros::ok()) {
        ros::spinOnce();
        wayfinder.wayfind();
    }
    return 1;
}
