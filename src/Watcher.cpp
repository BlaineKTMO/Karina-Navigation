/**
* Author: Blaine Oania
* File: Watcher.cpp
* Date: 3/31/2023
* Description:
*   Watcher node to monitor robot position and sensors.
*/

#include "Watcher.h"

void Watcher::checkLaserCollision(const sensor_msgs::LaserScanConstPtr& msg) {
    for(auto laser : msg->ranges) {
        ROS_WARN("%f", laser);
        if (laser < MIN_DISTANCE)
            stop = true;
            break;
    }
}

void Watcher::monitor() {
    while(ros::ok()) {
        if (stop){
            std_msgs::Bool msg;
            msg.data = true;
            stopPub.publish(msg);
        }
    }
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "Watcher");

    Watcher watcher;

    ros::spin();
    return 0;
}
