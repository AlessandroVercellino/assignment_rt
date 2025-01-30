#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <string>
#include <cmath>

// Global variable to store the pose of turtle1
turtlesim::Pose turtle1_pose;

// Callback to update the pose of turtle1
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1_pose = *msg;
}

// Function to calculate the distance between two points
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Subscribe to the pose of turtle1
    ros::Subscriber turtle1_pose_sub = nh.subscribe("turtle1/pose", 10, turtle1PoseCallback);

    // Wait for the spawn service to become available
    ros::service::waitForService("spawn");

    // Create a client for the spawn service
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;

    // Set the name of the second turtle
    spawn_srv.request.name = "turtle2";

    // Ensure a valid spawn location for turtle2
    float min_distance = 2.0; // Minimum distance from turtle1
    bool valid_position = false;

    while (!valid_position && ros::ok()) {
        // Propose a random position for turtle2
        spawn_srv.request.x = 2.0 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (8.0 - 2.0)));
        spawn_srv.request.y = 2.0 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (8.0 - 2.0)));

        // Check the distance between turtle1 and the proposed position
        if (calculateDistance(turtle1_pose.x, turtle1_pose.y, spawn_srv.request.x, spawn_srv.request.y) >= min_distance) {
            valid_position = true; // Position is valid
        }

        ros::spinOnce(); // Process the pose update for turtle1
    }

    // Spawn the second turtle
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 successfully at (%.2f, %.2f).", spawn_srv.request.x, spawn_srv.request.y);
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
        return 1;
    }

    // Create publishers for turtle1 and turtle2 velocity commands
    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle2_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    while (ros::ok()) {
        std::string turtle; // User selects which turtle to control
        float linear_x, linear_y, angular_z;

        // Ask the user which turtle to control
        std::cout << "Select turtle (turtle1 or turtle2): ";
        std::cin >> turtle;

        // Ask the user to specify the velocities
        std::cout << "Enter linear velocity along X: ";
        std::cin >> linear_x;
        std::cout << "Enter linear velocity along Y: ";
        std::cin >> linear_y;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular_z;

        // Create a Twist message to send the velocities
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_x;
        cmd_vel.linear.y = linear_y;
        cmd_vel.angular.z = angular_z;

        // Publish the command to the selected turtle
        if (turtle == "turtle1") {
            turtle1_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle1: linear_x=%.2f, linear_y=%.2f, angular=%.2f",
                     linear_x, linear_y, angular_z);
        } else if (turtle == "turtle2") {
            turtle2_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle2: linear_x=%.2f, linear_y=%.2f, angular=%.2f",
                     linear_x, linear_y, angular_z);
        } else {
            std::cout << "Invalid turtle name. Try again.\n";
            continue;
        }

        // Allow the turtle to move for 1 second
        ros::Duration(1.0).sleep();

        // Stop the turtle after 1 second
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        if (turtle == "turtle1") {
            turtle1_pub.publish(cmd_vel);
        } else {
            turtle2_pub.publish(cmd_vel);
        }
    }

    return 0;
}

