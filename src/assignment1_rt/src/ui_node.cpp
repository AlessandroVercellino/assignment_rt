#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Pose.h"
#include <iostream>
#include <string>
#include <cstdlib> // Per rand() e srand()
#include <ctime>   // Per time()

// Posizione della prima tartaruga
turtlesim::Pose turtle1_pose;

// Callback per aggiornare la posizione di turtle1
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr &msg) {
    turtle1_pose = *msg;
    ROS_INFO("Turtle1 Pose updated: x=%.2f, y=%.2f", turtle1_pose.x, turtle1_pose.y);
}

// Funzione per generare una posizione casuale
bool generateSafePosition(float &x, float &y, const turtlesim::Pose &turtle1_pose, float min_distance) {
    const float min_x = 2.0;
    const float max_x = 8.0;
    const float min_y = 2.0;
    const float max_y = 8.0;

    for (int attempt = 0; attempt < 100; ++attempt) {
        x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - min_x)));
        y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - min_y)));

        float distance = sqrt(pow(x - turtle1_pose.x, 2) + pow(y - turtle1_pose.y, 2));
        if (distance >= min_distance) {
            return true;
        }
    }

    ROS_WARN("Failed to generate a safe position for turtle2 after 100 attempts.");
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Sottoscrizione alla posizione della prima tartaruga
    ros::Subscriber turtle1_pose_sub = nh.subscribe("turtle1/pose", 10, turtle1PoseCallback);

    // Aspetta che la posizione di turtle1 sia aggiornata
    ROS_INFO("Waiting for turtle1 pose...");
    ros::Rate rate(10);
    while (ros::ok() && turtle1_pose.x == 0.0 && turtle1_pose.y == 0.0) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Turtle1 Pose received: x=%.2f, y=%.2f", turtle1_pose.x, turtle1_pose.y);

    // Spawn della seconda tartaruga (turtle2)
    ros::service::waitForService("spawn");
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;

    // Genera una posizione sicura per turtle2
    float turtle2_x, turtle2_y;
    float min_distance = 3.0; // Distanza minima tra turtle1 e turtle2
    srand(time(0));           // Inizializza il generatore di numeri casuali

    if (generateSafePosition(turtle2_x, turtle2_y, turtle1_pose, min_distance)) {
        spawn_srv.request.x = turtle2_x;
        spawn_srv.request.y = turtle2_y;
        spawn_srv.request.name = "turtle2";

        if (spawn_client.call(spawn_srv)) {
            ROS_INFO("Spawned turtle2 at x=%.2f, y=%.2f", turtle2_x, turtle2_y);
        } else {
            ROS_ERROR("Failed to spawn turtle2.");
            return 1;
        }
    } else {
        ROS_ERROR("Could not find a safe position to spawn turtle2.");
        return 1;
    }

    // Publisher per i nuovi topic sicuri
    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel_safe", 10);
    ros::Publisher turtle2_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel_safe", 10);

    // Ciclo per controllare le tartarughe
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
            ROS_INFO("Published to turtle1 (safe): linear=%.2f, angular=%.2f", linear, angular);
        } else if (turtle == "turtle2") {
            turtle2_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle2 (safe): linear=%.2f, angular=%.2f", linear, angular);
        } else {
            std::cout << "Invalid turtle name. Try again.\n";
            continue;
        }

        // Pausa per evitare pubblicazioni continue
        ros::Duration(1.0).sleep();
    }

    return 0;
}




