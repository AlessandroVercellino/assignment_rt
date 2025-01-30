#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <cmath>
#include <string>
#include <cstdlib>

// Costanti per i parametri di sicurezza
const float MIN_SAFE_DISTANCE = 2.0;  // Distanza minima di sicurezza tra le tartarughe
const float WALL_MARGIN = 1.5;        // Distanza di sicurezza dai muri
const float MIN_XY = 1.0;             // Limite minimo del piano
const float MAX_XY = 10.0;            // Limite massimo del piano

// Struttura per rappresentare lo stato di ogni tartaruga
struct Turtle {
    double x = 0.0, y = 0.0, theta = 0.0;
    ros::Publisher publisher;
    std::string id;
};

// Array per tenere traccia di due tartarughe
Turtle turtles[2];

// Callback per aggiornare la posizione di turtle1
void updateTurtle1Pose(const turtlesim::Pose::ConstPtr& msg) {
    turtles[0].x = msg->x;
    turtles[0].y = msg->y;
    turtles[0].theta = msg->theta;
}

// Callback per aggiornare la posizione di turtle2
void updateTurtle2Pose(const turtlesim::Pose::ConstPtr& msg) {
    turtles[1].x = msg->x;
    turtles[1].y = msg->y;
    turtles[1].theta = msg->theta;
}

// Funzione per calcolare la distanza al quadrato tra due tartarughe
float computeDistanceSquared(const Turtle& t1, const Turtle& t2) {
    return pow(t1.x - t2.x, 2) + pow(t1.y - t2.y, 2);
}

// Genera una posizione sicura casuale
void findSafePosition(double& new_x, double& new_y) {
    Turtle tempTurtle;
    do {
        new_x = MIN_XY + (rand() / (RAND_MAX + 1.0)) * (MAX_XY - MIN_XY);
        new_y = MIN_XY + (rand() / (RAND_MAX + 1.0)) * (MAX_XY - MIN_XY);

        // Assegna i valori generati alla tartaruga temporanea
        tempTurtle.x = new_x;
        tempTurtle.y = new_y;

    } while (computeDistanceSquared(tempTurtle, turtles[1]) < 4.0 ||  // Assicura 2m di distanza
             new_x < WALL_MARGIN || new_x > (MAX_XY - WALL_MARGIN) ||
             new_y < WALL_MARGIN || new_y > (MAX_XY - WALL_MARGIN));
}

// Funzione per teletrasportare la tartaruga
bool teleportTurtle(Turtle& turtle) {
    ros::NodeHandle nh;
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle.id + "/teleport_absolute");
    turtlesim::TeleportAbsolute srv;

    // Trova una posizione sicura
    double safe_x, safe_y;
    findSafePosition(safe_x, safe_y);

    srv.request.x = safe_x;
    srv.request.y = safe_y;
    srv.request.theta = 0.0;  // Manteniamo l'orientamento iniziale

    if (teleport_client.call(srv)) {
        ROS_INFO("[%s] teleported to safe position: (%.2f, %.2f)", turtle.id.c_str(), safe_x, safe_y);
        return true;
    } else {
        ROS_ERROR("Failed to teleport [%s]!", turtle.id.c_str());
        return false;
    }
}

// Funzione per gestire la collisione tra due tartarughe
void preventCollision(Turtle& t1, Turtle& t2) {
    float dist_squared = computeDistanceSquared(t1, t2);
    if (dist_squared < pow(MIN_SAFE_DISTANCE, 2)) {  // Se la distanza è inferiore a 2 metri
        ROS_WARN("[%s] and [%s] are too close! Teleporting to a safe position...", t1.id.c_str(), t2.id.c_str());

        teleportTurtle(t1);
        teleportTurtle(t2);
    }
}

// Funzione per gestire la vicinanza ai muri
void steerAwayFromWalls(Turtle& turtle) {
    if (turtle.x < WALL_MARGIN || turtle.x > (MAX_XY - WALL_MARGIN) ||
        turtle.y < WALL_MARGIN || turtle.y > (MAX_XY - WALL_MARGIN)) {
        
        ROS_WARN("[%s] too close to a wall! Teleporting to safety...", turtle.id.c_str());
        teleportTurtle(turtle);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_controller");
    ros::NodeHandle nh;

    // Subscriber per aggiornare le posizioni delle tartarughe
    ros::Subscriber sub_t1_pose = nh.subscribe("/turtle1/pose", 10, updateTurtle1Pose);
    ros::Subscriber sub_t2_pose = nh.subscribe("/turtle2/pose", 10, updateTurtle2Pose);

    // Configurazione delle tartarughe
    turtles[0].id = "turtle1";
    turtles[0].publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    turtles[1].id = "turtle2";
    turtles[1].publisher = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    // Inizializza il generatore di numeri casuali
    srand(time(0));

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // Controllo delle collisioni
        preventCollision(turtles[0], turtles[1]);

        // Controllo dei muri
        steerAwayFromWalls(turtles[0]);
        steerAwayFromWalls(turtles[1]);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}






