
#include "ros/ros.h"
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
}

// Callback per aggiornare la posizione di turtle2
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle2_pose = *msg;
}

// Callback per intercettare i comandi inviati a turtle1
void turtle1CmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    turtle1_cmd = *msg;
}

// Callback per intercettare i comandi inviati a turtle2
void turtle2CmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    turtle2_cmd = *msg;
}

// Calcola la distanza tra le due tartarughe
float calculateDistance(const turtlesim::Pose &pose1, const turtlesim::Pose &pose2) {
    return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
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

    // Publisher per pubblicare comandi filtrati
    ros::Publisher turtle1_cmd_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle2_cmd_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    ros::Rate rate(20); // Frequenza del ciclo principale (20 Hz)

    // Limiti per il muro virtuale
    const float min_x = 2.0;
    const float max_x = 8.0;
    const float min_y = 2.0;
    const float max_y = 8.0;

    // Distanza minima tra le due tartarughe
    const float min_distance = 1.0;

    while (ros::ok()) {
        // Calcola la distanza attuale tra le due tartarughe
        float distance = calculateDistance(turtle1_pose, turtle2_pose);

        // Log delle posizioni e della distanza
        ROS_INFO("Turtle1: x=%.2f, y=%.2f | Turtle2: x=%.2f, y=%.2f | Distance=%.2f",
                 turtle1_pose.x, turtle1_pose.y, turtle2_pose.x, turtle2_pose.y, distance);

        // Filtra i comandi per turtle1 (evita i bordi)
        if ((turtle1_pose.x <= min_x && turtle1_cmd.linear.x < 0.0) || 
            (turtle1_pose.x >= max_x && turtle1_cmd.linear.x > 0.0)) {
            ROS_WARN("Turtle1: Command blocked on X boundary.");
            turtle1_cmd.linear.x = 0.0;
        }
        if ((turtle1_pose.y <= min_y && turtle1_cmd.linear.x < 0.0) || 
            (turtle1_pose.y >= max_y && turtle1_cmd.linear.x > 0.0)) {
            ROS_WARN("Turtle1: Command blocked on Y boundary.");
            turtle1_cmd.linear.x = 0.0;
        }

        // Filtra i comandi per turtle2 (evita i bordi)
        if ((turtle2_pose.x <= min_x && turtle2_cmd.linear.x < 0.0) || 
            (turtle2_pose.x >= max_x && turtle2_cmd.linear.x > 0.0)) {
            ROS_WARN("Turtle2: Command blocked on X boundary.");
            turtle2_cmd.linear.x = 0.0;
        }
        if ((turtle2_pose.y <= min_y && turtle2_cmd.linear.x < 0.0) || 
            (turtle2_pose.y >= max_y && turtle2_cmd.linear.x > 0.0)) {
            ROS_WARN("Turtle2: Command blocked on Y boundary.");
            turtle2_cmd.linear.x = 0.0;
        }

        // Filtra i comandi per evitare che le tartarughe si avvicinino troppo
        if (distance < min_distance) {
            ROS_WARN("Turtles too close! Blocking commands to prevent collision.");
            
            // Blocca turtle1 se si muove verso turtle2
            if ((turtle1_pose.x < turtle2_pose.x && turtle1_cmd.linear.x > 0.0) ||
                (turtle1_pose.x > turtle2_pose.x && turtle1_cmd.linear.x < 0.0) ||
                (turtle1_pose.y < turtle2_pose.y && turtle1_cmd.linear.x > 0.0) ||
                (turtle1_pose.y > turtle2_pose.y && turtle1_cmd.linear.x < 0.0)) {
                turtle1_cmd.linear.x = 0.0;
                ROS_WARN("Turtle1 movement blocked due to proximity to Turtle2.");
            }

            // Blocca turtle2 se si muove verso turtle1
            if ((turtle2_pose.x < turtle1_pose.x && turtle2_cmd.linear.x > 0.0) ||
                (turtle2_pose.x > turtle1_pose.x && turtle2_cmd.linear.x < 0.0) ||
                (turtle2_pose.y < turtle1_pose.y && turtle2_cmd.linear.x > 0.0) ||
                (turtle2_pose.y > turtle1_pose.y && turtle2_cmd.linear.x < 0.0)) {
                turtle2_cmd.linear.x = 0.0;
                ROS_WARN("Turtle2 movement blocked due to proximity to Turtle1.");
            }
        }

        // Pubblica i comandi filtrati
        turtle1_cmd_pub.publish(turtle1_cmd);
        turtle2_cmd_pub.publish(turtle2_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


