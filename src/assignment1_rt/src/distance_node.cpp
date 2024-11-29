#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <string>

// Costanti per i parametri di sicurezza
const float MIN_SAFE_DISTANCE = 1;  // Distanza minima di sicurezza tra le tartarughe
const float WALL_MARGIN = 1.5;        // Distanza di sicurezza dai muri
const float MIN_XY = 1.0;             // Limite minimo del piano
const float MAX_XY = 10.0;            // Limite massimo del piano

// Struttura per rappresentare lo stato di ogni tartaruga
struct Turtle {
    double x = 0.0, y = 0.0, theta = 0.0;
    ros::Publisher publisher;
    std::string id;
    bool is_active = false; // Indica se la tartaruga si sta muovendo
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

// Funzione per fermare una tartaruga
void haltTurtle(Turtle& turtle) {
    geometry_msgs::Twist halt_cmd;
    halt_cmd.linear.x = 0.0;
    halt_cmd.linear.y = 0.0;
    halt_cmd.angular.z = 0.0;

    // Invia il comando di stop
    turtle.publisher.publish(halt_cmd);
    turtle.is_active = false;

    // Log dettagliato
    ROS_INFO("[%s] has been stopped. Command sent: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
             turtle.id.c_str(), halt_cmd.linear.x, halt_cmd.linear.y, halt_cmd.angular.z);
}

// Funzione per gestire la distanza dai muri
void steerAwayFromWalls(Turtle& turtle) {
    // Margine per avvicinarsi ai muri
    const float WALL_MARGIN = 0.8;

    // Verifica se la tartaruga è troppo vicina ai bordi
    if (turtle.x < WALL_MARGIN || turtle.x > (MAX_XY - WALL_MARGIN) ||
        turtle.y < WALL_MARGIN || turtle.y > (MAX_XY - WALL_MARGIN)) {
        
        ROS_WARN("[%s] is too close to the wall. Adjusting position...", turtle.id.c_str());

        geometry_msgs::Twist reverse_cmd;

        // Movimento di allontanamento dai muri
        reverse_cmd.linear.x = 0.0;
        reverse_cmd.linear.y = 0.0;

        if (turtle.x < WALL_MARGIN) {
            reverse_cmd.linear.x = 0.5;  // Muovi verso destra
        } else if (turtle.x > (MAX_XY - WALL_MARGIN)) {
            reverse_cmd.linear.x = -0.5; // Muovi verso sinistra
        }

        if (turtle.y < WALL_MARGIN) {
            reverse_cmd.linear.y = 0.5;  // Muovi verso l'alto
        } else if (turtle.y > (MAX_XY - WALL_MARGIN)) {
            reverse_cmd.linear.y = -0.5; // Muovi verso il basso
        }

        reverse_cmd.angular.z = 0.0; // Nessuna rotazione

        // Pubblica il comando di movimento
        turtle.publisher.publish(reverse_cmd);
        ros::Duration(0.2).sleep(); // Movimento breve

        // Ferma la tartaruga dopo il movimento
        haltTurtle(turtle);
        ROS_INFO("[%s] has been repositioned away from the wall.", turtle.id.c_str());
    }
}




// Funzione per calcolare la distanza al quadrato tra due tartarughe
float computeDistanceSquared(const Turtle& t1, const Turtle& t2) {
    return pow(t1.x - t2.x, 2) + pow(t1.y - t2.y, 2);
}

// Funzione per gestire la collisione tra due tartarughe
void preventCollision(Turtle& t1, Turtle& t2) {
    float dist_squared = computeDistanceSquared(t1, t2);
    if (dist_squared < pow(MIN_SAFE_DISTANCE, 2)) {
        float actual_distance = sqrt(dist_squared);
        ROS_WARN("WARNING: [%s] and [%s] are too close! Current distance: %.2f. Taking corrective action.", t1.id.c_str(), t2.id.c_str(), actual_distance);

        // Ferma entrambe le tartarughe
        haltTurtle(t1);
        haltTurtle(t2);

        // Movimento di separazione
        geometry_msgs::Twist separation_cmd;
        separation_cmd.linear.x = 0.5;
        separation_cmd.linear.y = 0.5;
        separation_cmd.angular.z = 0.0;
        t1.publisher.publish(separation_cmd);

        separation_cmd.linear.x = -0.5;
        separation_cmd.linear.y = -0.5;
        t2.publisher.publish(separation_cmd);

        ros::Duration(0.5).sleep();
        haltTurtle(t1);
        haltTurtle(t2);

        // Log della distanza aggiornata
        float new_distance = sqrt(computeDistanceSquared(t1, t2));
        ROS_INFO("Safe distance restored. New distance between [%s] and [%s]: %.2f.", t1.id.c_str(), t2.id.c_str(), new_distance);
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

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // Gestione delle collisioni tra le tartarughe
        preventCollision(turtles[0], turtles[1]);

        // Controllo dei confini per ciascuna tartaruga
        steerAwayFromWalls(turtles[0]);
        steerAwayFromWalls(turtles[1]);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


