#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cmath>

// Variabili globali per memorizzare le pose delle tartarughe
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
geometry_msgs::Twist turtle1_cmd;
geometry_msgs::Twist turtle2_cmd;

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

// Callback per intercettare i comandi inviati a turtle1
void turtle1CmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    turtle1_cmd = *msg;
    ROS_INFO("Intercepted command for turtle1: linear=%.2f, angular=%.2f",
             turtle1_cmd.linear.x, turtle1_cmd.angular.z);
}

// Callback per intercettare i comandi inviati a turtle2
void turtle2CmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    turtle2_cmd = *msg;
    ROS_INFO("Intercepted command for turtle2: linear=%.2f, angular=%.2f",
             turtle2_cmd.linear.x, turtle2_cmd.angular.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    // Sottoscrizioni alle pose delle tartarughe
    ros::Subscriber turtle1_pose_sub = nh.subscribe("turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_pose_sub = nh.subscribe("turtle2/pose", 10, turtle2PoseCallback);

    // Sottoscrizioni per intercettare i comandi inviati
    ros::Subscriber turtle1_cmd_sub = nh.subscribe("turtle1/cmd_vel", 10, turtle1CmdCallback);
    ros::Subscriber turtle2_cmd_sub = nh.subscribe("turtle2/cmd_vel", 10, turtle2CmdCallback);

    // Publisher per controllare le tartarughe
    ros::Publisher turtle1_cmd_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel_safe", 10);
    ros::Publisher turtle2_cmd_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel_safe", 10);

    // Publisher per pubblicare la distanza tra le tartarughe
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 10);

    ros::Rate rate(10); // Frequenza del ciclo principale

    // Soglie per i confini
    const float safety_margin = 2.5;
    const float min_x = safety_margin;
    const float max_x = 10.0 - safety_margin;
    const float min_y = safety_margin;
    const float max_y = 10.0 - safety_margin;

    while (ros::ok()) {
        // Calcola la distanza tra le due tartarughe
        float distance = sqrt(pow(turtle1_pose.x - turtle2_pose.x, 2) +
                              pow(turtle1_pose.y - turtle2_pose.y, 2));

        // Pubblica la distanza calcolata
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);
        ROS_INFO("Distance between turtles: %.2f", distance);

        // Filtra i comandi per turtle1
        if ((turtle1_pose.x <= min_x && turtle1_cmd.linear.x < 0.0) || 
            (turtle1_pose.x >= max_x && turtle1_cmd.linear.x > 0.0) ||
            (turtle1_pose.y <= min_y && turtle1_cmd.linear.x < 0.0) ||
            (turtle1_pose.y >= max_y && turtle1_cmd.linear.x > 0.0)) {
            ROS_WARN("Command to turtle1 blocked due to boundary.");
            turtle1_cmd.linear.x = 0.0;
            turtle1_cmd.angular.z = 0.0;
        } else {
            ROS_INFO("Turtle1 command allowed: linear=%.2f, angular=%.2f", 
                     turtle1_cmd.linear.x, turtle1_cmd.angular.z);
        }

        // Filtra i comandi per turtle2
        if ((turtle2_pose.x <= min_x && turtle2_cmd.linear.x < 0.0) || 
            (turtle2_pose.x >= max_x && turtle2_cmd.linear.x > 0.0) ||
            (turtle2_pose.y <= min_y && turtle2_cmd.linear.x < 0.0) ||
            (turtle2_pose.y >= max_y && turtle2_cmd.linear.x > 0.0)) {
            ROS_WARN("Command to turtle2 blocked due to boundary.");
            turtle2_cmd.linear.x = 0.0;
            turtle2_cmd.angular.z = 0.0;
        } else {
            ROS_INFO("Turtle2 command allowed: linear=%.2f, angular=%.2f", 
                     turtle2_cmd.linear.x, turtle2_cmd.angular.z);
        }

        // Verifica la distanza tra le tartarughe
        if (distance < 1.0) {
            ROS_WARN("Turtles are too close! Stopping both turtles.");
            turtle1_cmd.linear.x = 0.0;
            turtle1_cmd.angular.z = 0.0;
            turtle2_cmd.linear.x = 0.0;
            turtle2_cmd.angular.z = 0.0;
        }

        // Pubblica i comandi filtrati
        turtle1_cmd_pub.publish(turtle1_cmd);
        turtle2_cmd_pub.publish(turtle2_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


