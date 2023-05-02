/**
* Author: Blaine Oania
* File: Wayfinder.cpp
* Date: 3/31/2023
* Description:
*   Subsumptive local navigator node.
*/

#include <ros/ros.h>
#include <Wayfinder.h>

void Wayfinder::vfhWptCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    vfhWpt = *msg.get(); 
}

void Wayfinder::lfWptCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    lfWpt = *msg.get();
    addToPath(lfWpt);
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

void Wayfinder::reverseCallback(const std_msgs::BoolConstPtr& msg) {
    reverseStatus = true;
}

/**
 * @brief Reverse recovery action
 */
void Wayfinder::reverse() {
    geometry_msgs::Twist msg;
    msg.linear.x = -2;
    velPub.publish(msg);
}

/** 
 * @brief Navigation function which checks robot status and acts accordingly. 
 */
void Wayfinder::navigate() {
    rate.sleep();
    // If collision is detected
    // Early return to avoid wayfind
    if(reverseStatus) {
        ROS_WARN("Reversing");
        reverse();
        reverseStatus = false;
        return;
    }

    // Stop the robot
    if(stopStatus) {
        // stopStatus = false;

        // Probably better to publish here
        targetVelocity.linear.x = 0; // Set v to zero, wayfind loop will publish
    }
    
    if(collectedPath.size() > 40) {
        // Publish path
    }
    wayfind();
}

/**
 * @brief Main wayfind loop. Checks waypoint status, and if available, sets waypoint
 * to new read waypoint.
 */
void Wayfinder::wayfind() {
    // wpt = getCurrentPos();
   
    // Set default move forward command
    geometry_msgs::PoseStamped forward;
    forward.header.frame_id = "base_link";
    forward.pose.position.x = 1.5;
    forward.pose.orientation.w = 1;


    // wpt.header.frame_id = "base_link";
    // wpt.pose.position.x = 1.5;
    // wpt.pose.position.y = 0;
    // wpt.pose.position.z = 0;
    // wpt.pose.orientation.x = 0;
    // wpt.pose.orientation.y = 0;
    // wpt.pose.orientation.z = 0;
    // wpt.pose.orientation.w = 1;

    tfBuffer.transform<geometry_msgs::PoseStamped>(forward, "map", ros::Duration(2));
    wpt = forward;

    // Prepare goal pos for move_base path service
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
   
    // Publish target velocity and wpt for dwa controller
    targetVelocityPub.publish(targetVelocity);
    wptPub.publish(wpt);
    // ROS_WARN("%.3f, %.3f", wpt.pose.position.x, wpt.pose.position.y);
}

void Wayfinder::addToPath(geometry_msgs::PoseStamped pose) {
    if (getDistance(pose.pose, collectedPath.back().pose) > 1) {
        collectedPath.clear();
    }

    collectedPath.push_back(pose);
    if (collectedPath.size() > 50) {
        std::reverse(std::begin(collectedPath), std::end(collectedPath));
        collectedPath.pop_back();
        std::reverse(std::begin(collectedPath), std::end(collectedPath));
    }

    for (auto iter : collectedPath) {
        path.poses.push_back(iter);
    }
}

float Wayfinder::getDistance(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB) {
    return sqrt(pow(poseA.position.x - poseB.position.x, 2) + pow(poseA.position.y - poseB.position.y, 2));
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

/**
 * @brief Retrieve a point from the move_base path service
 *
 * @param goal Goal position
 * @return Path at index 30
 */
geometry_msgs::PoseStamped Wayfinder::getFromPath(geometry_msgs::PoseStamped goal) 
{
    // Path service request message
    nav_msgs::GetPlan path_service;
    path_service.request.goal = goal;
    path_service.request.start = getCurrentPos();
    path_service.request.tolerance = 3;

    geometry_msgs::PoseStamped ret;

    // Call service and parse response
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
    
    // Launch wayfinder and start navigation loop
    Wayfinder wayfinder;
    while(ros::ok()) {
        ros::spinOnce();
        wayfinder.navigate();
    }
    return 1;
}
