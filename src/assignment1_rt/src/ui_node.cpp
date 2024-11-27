#include "ros/ros.h"               // Include la libreria principale di ROS
#include "geometry_msgs/Twist.h"   // Include il messaggio per inviare velocità lineare e angolare
#include "turtlesim/Spawn.h"       // Include il servizio per creare una nuova tartaruga
#include <iostream>                // Include per input/output standard
#include <string>                  // Include per gestire stringhe

int main(int argc, char **argv) {
    // Inizializza il nodo ROS con il nome "ui_node"
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh; // Nodo ROS per gestire le comunicazioni

    // Aspetta che il servizio "/spawn" sia disponibile
    ros::service::waitForService("spawn");

    // Crea un client per il servizio "/spawn" per creare una nuova tartaruga
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn spawn_srv;

    // Configura la posizione e il nome della nuova tartaruga (turtle2)
    spawn_srv.request.x = 3.0;
    spawn_srv.request.y = 5.0;
    spawn_srv.request.name = "turtle2";

    // Chiama il servizio per creare "turtle2"
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Spawned turtle2 successfully."); // Log di successo
    } else {
        ROS_ERROR("Failed to spawn turtle2."); // Log di errore
        return 1; // Termina il programma se il servizio fallisce
    }

    // Crea due publisher per inviare comandi di velocità a turtle1 e turtle2
    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Publisher turtle2_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    // Ciclo principale per interagire con l'utente
    while (ros::ok()) {
        std::string turtle;      // Nome della tartaruga selezionata dall'utente
        float linear, angular;   // Velocità lineare e angolare inserite dall'utente

        // Chiede all'utente di selezionare quale tartaruga controllare
        std::cout << "Select turtle (turtle1 or turtle2): ";
        std::cin >> turtle;

        // Chiede all'utente di specificare la velocità lineare
        std::cout << "Enter linear velocity: ";
        std::cin >> linear;

        // Chiede all'utente di specificare la velocità angolare
        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        // Crea un messaggio di velocità da inviare alla tartaruga
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear;    // Imposta la velocità lineare
        cmd_vel.angular.z = angular; // Imposta la velocità angolare

        // Pubblica il comando sulla tartaruga selezionata
        if (turtle == "turtle1") {
            turtle1_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle1: linear=%.2f, angular=%.2f", linear, angular);
        } else if (turtle == "turtle2") {
            turtle2_pub.publish(cmd_vel);
            ROS_INFO("Published to turtle2: linear=%.2f, angular=%.2f", linear, angular);
        } else {
            // Se l'input dell'utente è errato, mostra un messaggio di errore
            std::cout << "Invalid turtle name. Try again.\n";
            continue; // Ripete il ciclo senza inviare comandi
        }

        // Mantiene il comando per 1 secondo
        ros::Duration(1.0).sleep();

        // Imposta le velocità a zero per fermare la tartaruga
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        // Ferma la tartaruga selezionata
        if (turtle == "turtle1") {
            turtle1_pub.publish(cmd_vel);
            ROS_INFO("Stopped turtle1.");
        } else if (turtle == "turtle2") {
            turtle2_pub.publish(cmd_vel);
            ROS_INFO("Stopped turtle2.");
        }
    }

    return 0; // Fine del programma
}





