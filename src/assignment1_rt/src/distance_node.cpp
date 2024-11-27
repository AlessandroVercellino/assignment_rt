#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

// Variabili globali per memorizzare le pose delle tartarughe
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

// Callback per aggiornare la posizione di turtle1
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1_pose = *msg;
    ROS_INFO("Turtle1 Pose updated: x=%.2f, y=%.2f", turtle1_pose.x, turtle1_pose.y);
}

// Callback per aggiornare la posizione di turtle2
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle2_pose = *msg;
    ROS_INFO("Turtle2 Pose updated: x=%.2f, y=%.2f", turtle2_pose.x, turtle2_pose.y);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Sottoscrizioni alle pose delle tartarughe
    ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 10, turtle2PoseCallback);

    // Publisher per fermare le tartarughe
    ros::Publisher turtle1_stop_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle2_stop_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    // Publisher per pubblicare la distanza tra le tartarughe
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 10);

    ros::Rate rate(10); // Frequenza del ciclo principale

    while (ros::ok()) {
        // Calcola la distanza tra le due tartarughe
        float distance = sqrt(pow(turtle1_pose.x - turtle2_pose.x, 2) +
                              pow(turtle1_pose.y - turtle2_pose.y, 2));

        // Pubblica la distanza calcolata
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);
        ROS_INFO("Distance between turtles: %.2f", distance);

        // Comandi di stop per turtle1 e turtle2
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;

        // Condizioni di fermo per turtle1
        if (distance < 1.0 || 
            turtle1_pose.x < 1.0 || turtle1_pose.x > 10.0 || 
            turtle1_pose.y < 1.0 || turtle1_pose.y > 10.0) {
            ROS_WARN("Stopping turtle1 due to proximity or boundary.");
            turtle1_stop_pub.publish(stop_cmd);
        }

        // Condizioni di fermo per turtle2
        if (distance < 1.0 || 
            turtle2_pose.x < 1.0 || turtle2_pose.x > 10.0 || 
            turtle2_pose.y < 1.0 || turtle2_pose.y > 10.0) {
            ROS_WARN("Stopping turtle2 due to proximity or boundary.");
            turtle2_stop_pub.publish(stop_cmd);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

