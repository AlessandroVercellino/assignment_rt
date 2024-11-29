Turtlesim Nodes: `ui_node` and `distance_node`

 Overview
This project includes two ROS nodes that interact with the Turtlesim simulator. The nodes, `ui_node` and `distance_node`, work together to control the behavior of two turtles (`turtle1` and `turtle2`), ensuring they avoid collisions with walls and each other. The nodes handle the spawning, movement, and safety logic of the turtles in a coordinated manner.


 `ui_node`
 Purpose
The `ui_node` allows the user to control the turtles' movement through a command-line interface. It provides options to select a turtle and specify linear and angular velocities along both the x and y axes.

 Key Features
1. Spawning the Second Turtle:
   - When the node starts, it spawns a second turtle (`turtle2`) in the Turtlesim environment at a position that ensures it is not too close to `turtle1`.

2. Interactive Movement:
   - Users can select a turtle (`turtle1` or `turtle2`) and specify:
     - Linear velocity along x and y axes.
     - Angular velocity for rotation.
   - The turtles execute the movement command for a fixed duration (1 second) and then stop automatically.

3. Input Validation:
   - Ensures that only valid turtle names (`turtle1` or `turtle2`) are accepted.
   - Prevents collisions by avoiding invalid commands or overlapping spawning positions.

 How It Works
- When the node starts, it spawns `turtle2` at a random position ensuring sufficient distance from `turtle1`.
- The user enters commands via the terminal to control the selected turtle.
- After executing the movement, the turtle halts and waits for the next command.



 `distance_node`
 Purpose
The `distance_node` ensures the safety of the turtles by monitoring their proximity to each other and the walls. It implements collision avoidance logic and moves the turtles backward when necessary.

 Key Features
1. Collision Avoidance Between Turtles:**
   - The node calculates the distance between `turtle1` and `turtle2` in real time.
   - If the distance falls below a predefined safety threshold, the turtles are moved apart.

2. Wall Avoidance Logic:
   - The node monitors the positions of both turtles relative to the Turtlesim boundaries.
   - When a turtle gets too close to a wall, it is moved slightly away to prevent collisions.

3. **Seamless Integration:**
   - The node works in the background to enforce safety without interrupting user commands from `ui_node`.

 How It Works
- The node subscribes to the `/turtle1/pose` and `/turtle2/pose` topics to get the real-time positions of the turtles.
- It calculates the distance between the turtles and checks their proximity to walls.
- If a collision risk is detected:
  - The turtles are moved backward (along x and y axes) until they are in a safe zone.
  - Once the adjustment is complete, the turtles are stopped and ready for further commands.



 Topics Used
 Subscribed Topics
- `/turtle1/pose`:
  - Provides the position and orientation of `turtle1`.
- `/turtle2/pose`:
  - Provides the position and orientation of `turtle2`.

Published Topics
- `/turtle1/cmd_vel`:
  - Sends velocity commands to `turtle1`.
- `/turtle2/cmd_vel`:
  - Sends velocity commands to `turtle2`.


 How to Run
 Prerequisites
- Ensure ROS (Noetic or compatible version) is installed.
- Turtlesim package should be available.
- Clone this repository into your ROS workspace.

Build and Launch
1. Build the workspace:
   
   cd <your_workspace>
   catkin_make
   
2. Source the workspace:
   source devel/setup.bash
   
3. Launch the Turtlesim simulator:
   rosrun turtlesim turtlesim_node
4. Run the `ui_node`:
   rosrun <your_package_name> ui_node
5. Run the `distance_node`:
   rosrun <your_package_name> distance_node
