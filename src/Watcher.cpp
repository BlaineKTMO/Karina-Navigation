/**
* Author: Blaine Oania
* File: Watcher.cpp
* Date: 3/31/2023
* Description:
*   Watcher node to monitor robot position and sensors.
*/

#include "Watcher.h"

void Watcher::checkLaserCollision(const sensor_msgs::LaserScanConstPtr& msg) {
    for(int i = 100; i < 120; i++) {
        // ROS_WARN("%f", msg->ranges[i]);
        // If obstacle directly in front of robot
        if (msg->ranges[i] < MIN_DISTANCE) {
            reversePub.publish(trueStdMsg);
            break;
        }
    }
}

/**
 * @brief Main watchdog loop. Checks status' from callbacks and prevents robot
 * navigation when necessary.
 */
void Watcher::monitor() {
    // If robot blocked
    // if (stop){
    //     ROS_INFO("Robot stopped!");
    //     std_msgs::Bool msg;
    //     msg.data = true;
    //     stopPub.publish(msg);
    //     reversePub.publish(msg);
    //
    //     stop = false;
    // }

    // Set target velocity for wayfinder
    targetVel.linear.x = 0.4;
    targetVelPub.publish(targetVel);

    rate.sleep();
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "Watcher");

    // Launch watcher and start monitoring loop
    Watcher watcher;
    while (ros::ok()) {
        watcher.monitor();
        ros::spinOnce();
    }

    return 0;
}
