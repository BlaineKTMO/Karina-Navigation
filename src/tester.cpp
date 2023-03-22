#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    ros::Rate rate(10);
    
    geometry_msgs::Twist target_vel;

    ros::Publisher vfhStatPub = n.advertise<std_msgs::Bool>("/vfh_status", 10);
    ros::Publisher lfStatPub = n.advertise<std_msgs::Bool>("/lf_status", 10);
    ros::Publisher targetVelPub = n.advertise<geometry_msgs::Twist>("/target_vel", 10);

    while(ros::ok()) {
        char input;
        std::cin >> input;
        
        std_msgs::Bool msg;
        msg.data = true;

        switch (tolower(input)) {
            case 'a':
                vfhStatPub.publish(msg);
                break;
            case 'd':
                lfStatPub.publish(msg);
                break;
            case 's':
                msg.data = false;
                lfStatPub.publish(msg);
                vfhStatPub.publish(msg);
                break;
            case 'w':
                target_vel.linear.x += .2;
                targetVelPub.publish(target_vel);
                break;
            case 'x':
                target_vel.linear.x -= .2;
                targetVelPub.publish(target_vel);
                break;
            default:
                std::cout << "Please input A or D" << std::endl;
                break;

        }

        rate.sleep();
        ros::spinOnce();

    }
    
    return 1;
}
