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
    if (stop){
        std_msgs::Bool msg;
        msg.data = true;
        stopPub.publish(msg);
    }
    rate.sleep();
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "Watcher");

    Watcher watcher;
    while (ros::ok()) {
        watcher.monitor();
        ros::spinOnce();
    }

    return 0;
}
