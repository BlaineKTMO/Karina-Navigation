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

void Wayfinder::targetVelocityCallback(const geometry_msgs::TwistConstPtr& msg) {
    targetVelocity = *msg.get();
} 

void Wayfinder::wayfind() {
    wpt.header.frame_id = "map";
    wpt.pose.position = getCurrentPos();
    wpt.pose.position.x += 5;

    wpt.pose.orientation.x = 0;
    wpt.pose.orientation.y = 0;
    wpt.pose.orientation.z = 0;
    wpt.pose.orientation.w = 1;
   
    if(vfhStatus) {
        // wpt = vfhWpt;
        wpt.pose.position.y += 5;
    }
    if(lfStatus) {
        // wpt = lfWpt;
        wpt.pose.position.y -= 5;
    }
    
    targetVelocityPub.publish(targetVelocity);
    wptPub.publish(wpt);
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
    while(ros::ok()) {
        ros::spinOnce();
        wayfinder.wayfind();
    }
    return 1;
}
