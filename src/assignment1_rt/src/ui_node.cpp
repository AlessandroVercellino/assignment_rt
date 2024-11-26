#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Spawn a new turtle (turtle2)
    ros::service::waitForService("spawn");
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.0;
    spawn_srv.request.y = 5.0;
    spawn_srv.request.name = "turtle2";

    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 successfully.");
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
        return 1;
    }

    // Publishers for turtle1 and turtle2
    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle2_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    while (ros::ok()) {
        std::string turtle;
        float linear, angular;

        std::cout << "Select turtle (turtle1 or turtle2): ";
        std::cin >> turtle;
        std::cout << "Enter linear velocity: ";
        std::cin >> linear;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;

        if (turtle == "turtle1") {
            turtle1_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle1: linear=%.2f, angular=%.2f", linear, angular);
        } else if (turtle == "turtle2") {
            turtle2_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle2: linear=%.2f, angular=%.2f", linear, angular);
        } else {
            std::cout << "Invalid turtle name. Try again.\n";
            continue;
        }

        // Pausa di 1 secondo per evitare pubblicazioni continue
        ros::Duration(1.0).sleep();
    }

    return 0;
}

