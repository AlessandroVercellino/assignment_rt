#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1_pose = *msg;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle2_pose = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 10, turtle2PoseCallback);

    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 10);
    ros::Publisher turtle2_stop_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(10);

    while (ros::ok()) {
        float distance = sqrt(pow(turtle1_pose.x - turtle2_pose.x, 2) +
                              pow(turtle1_pose.y - turtle2_pose.y, 2));

        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);

        // Log della distanza
        ROS_INFO("Distance between turtles: %.2f", distance);

        if (distance < 1.0) {
            ROS_WARN("Turtles are too close! Stopping turtle2.");
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            turtle2_stop_pub.publish(stop_cmd);
        }

        if (turtle2_pose.x > 10.0 || turtle2_pose.x < 1.0 || 
            turtle2_pose.y > 10.0 || turtle2_pose.y < 1.0) {
            ROS_WARN("Turtle2 is near the boundary! Stopping turtle2.");
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            turtle2_stop_pub.publish(stop_cmd);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


