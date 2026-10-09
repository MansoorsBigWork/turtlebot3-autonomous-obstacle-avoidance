TurtleBot3 (Burger) Autonomous Obstacle 
Avoidance 
Simulation using ROS2 and Gazebo 
1. Introduction 
This report describes the development of an autonomous obstacle avoidance controller for the TurtleBot3 
(Burger) robot using ROS2 and Gazebo. The aim of the assignment is to make the robot move continuously 
around a simulated environment, detect nearby obstacles with its LiDAR sensor, stop before collision, and 
then rotate in the safer direction before continuing forward motion. 
The final behaviour is based on a simple reactive control algorithm rather than map-based navigation. This 
means the robot does not build a map of the world or plan an optimal route in advance. Instead, it 
constantly reads live laser data, decides whether the area directly ahead is safe, and immediately updates 
its motion command. This approach is suitable for a robot vacuum style task because it is lightweight, 
responsive, and relatively straightforward to implement in C++. 
The completed system was tested in Gazebo using the supplied world and launch file. The robot was able 
to move through the environment for a prolonged period without crashing, while also choosing to turn left 
or right depending on which side offered more free space. 
2. Background: ROS2 and Gazebo 
ROS2 
ROS2 provides the communication framework used by the robot software. Each major function can be 
separated into nodes, and those nodes communicate through topics. In this assignment the control node 
subscribes to LiDAR scan data from the scan topic and publishes velocity commands to cmd_vel. ROS2 
also provides timing, execution control, quality of service settings, and message definitions such as 
sensor_msgs/msg/LaserScan and geometry_msgs/msg/TwistStamped. 
Gazebo 
Gazebo is used to simulate both the robot and its environment. It provides a realistic physics engine and 
sensor models, allowing the controller to be tested safely without physical hardware. The TurtleBot3 Burger 
model includes a laser scanner, so the obstacle avoidance logic can be developed and observed visually 
in the simulator before it would ever be deployed on a real mobile robot. 
Why this combination is useful. 
Using ROS2 together with Gazebo allows the same control ideas used in real robotics to be implemented 
in a repeatable virtual environment. It also makes debugging easier because sensor behaviour, robot 
motion and obstacle placement can all be observed clearly while the code runs. 
3. Methodology and Code Implementation 
The controller is implemented as a C++ ROS2 node called turtlebot3_driver. The code is split across a 
header file, an implementation file and a main file. The header declares the movement functions, distance 
getters, laser callback and internal variables. The implementation file processes LiDAR measurements 
and publishes movement commands. The main file runs the decision loop at 10 Hz, which means the robot 
re-evaluates its environment ten times per second. 
3.1 How the code makes the robot move. 
Three movement functions are defined inside the driver class: moveForward(), stopMoving() and 
turn(double angular). Each function creates a geometry_msgs::msg::TwistStamped message, adds a time 
stamp, sets the robot frame to base_link, and then publishes the command to the cmd_vel topic. 
Forward motion is produced by setting linear.x to a positive value while keeping angular.z equal to zero. 
In this code the forward speed is fixed at 0.2 m/s. This means the robot drives straight ahead whenever 
the path is considered safe. 
Stopping is produced by setting both linear.x and angular.z to zero. This prevents the robot from continuing 
into an obstacle. 
Turning is produced by setting linear.x to zero and applying an angular velocity about the z-axis. In the 
main decision loop the code turns left with +0.8 rad/s and turns right with −0.8 rad/s. Positive angular.z 
rotates the robot anticlockwise, while negative angular.z rotates it clockwise. 
3.2 LiDAR detection distances and angle regions. 
The scanCallback() function processes the LiDAR data. A minimum obstacle threshold of 0.7 m is used. If 
the measured front distance drops below 0.7 m, the boolean flag isObstacleDetected becomes true and 
the robot switches from forward motion to avoidance behaviour. 
The laser scan is divided into angular regions measured in degrees. The front region is formed from two 
smaller sectors: front-left from −35° to 0° and front-right from 0° to +35°. The smaller of these two averages 
is taken as the front distance. This is a sensible choice because even if only one side of the front is blocked, 
the robot should still treat the path ahead as unsafe. 
Side detection is also based on averaged LiDAR values. The left side is measured from +35° to +135°, 
and the right side is measured from −135° to −35°. These wider side sectors allow the robot to estimate 
which side has more open space before rotating. 
The code only uses valid LiDAR readings. Infinite values and measurements outside the sensor range are 
ignored. If no valid reading exists in a region, the code returns infinity, which effectively means that region 
is treated as clear. 
3.3 Key control parameters taken from the code. 
Parameter 
Value 
Forward speed 
0.2 m/s 
Explanation 
Applied in moveForward() by 
setting linear.x = 0.2 and 
angular.z = 0.0. 
Left turn speed +0.8 rad/s Applied in main.cpp when 
leftDistance > rightDistance. 
Right turn speed −0.8 rad/s 
Applied in main.cpp when 
rightDistance is greater or equal 
to leftDistance. 
Obstacle threshold 0.7 m 
If frontDistance < 0.7, the robot 
is considered too close to an 
obstacle. 
Front-left region −35° to 0° 
Used in averageRange() to 
estimate one half of the forward 
path. 
Front-right region 0° to +35° 
Used in averageRange() to 
estimate the other half of the 
forward path. 
Left side region +35° to +135° Used to judge free space 
available on the left. 
Right side region −135° to −35° Used to judge free space 
available on the right. 
Control loop rate 10 Hz The main loop runs with 
rclcpp::Rate loop_rate(10). 
 
3.4 How side detection affects the decision. 
Once an obstacle has been detected ahead, the robot compares the averaged distance on the left side 
with the averaged distance on the right side. This comparison is performed in the main loop using 
getLeftDistance() and getRightDistance(). 
If the left distance is greater than the right distance, then the left side is more open, so the robot turns left 
at +0.8 rad/s. Otherwise it turns right at −0.8 rad/s. This gives the robot a simple but effective form of 
directional intelligence, because it does not always turn the same way when an obstacle appears. 
Averaging over broad side sectors reduces the effect of a single noisy reading and gives a more stable 
estimate of free space. That makes the turning decision smoother and improves the chance of escaping 
corners or cluttered areas. 
3.5 code that displays operation. 
const double MIN_DIST_FROM_OBSTACLE = 0.7; 
front_left  = averageRange(scan, -35.0, 0.0); 
front_right = averageRange(scan,  0.0, 35.0); 
frontDistance = std::min(front_left, front_right); 
leftDistance  = averageRange(scan,  35.0, 135.0); 
rightDistance = averageRange(scan, -135.0, -35.0); 
isObstacleDetected = (frontDistance < MIN_DIST_FROM_OBSTACLE); 
if (!node->obstacleDetected()) { 
    node->moveForward(); 
} else { 
    node->stopMoving(); 
    if (node->getLeftDistance() > node->getRightDistance()) { 
node->turn(0.8); 
} else { 
node->turn(-0.8); 
} 
} 
4. Navigation Logic and Flowchart. 
The navigation strategy can be summarised as a repeated sense–decide–act loop. First, the node receives 
new LiDAR data. Second, it checks whether the front path is safe by comparing the front distance with the 
0.7 m threshold. Third, it sends either a forward motion command or a turn command. After that, the 
process immediately repeats in the next control cycle. 
Because the loop returns to reading LiDAR data after every action, the robot is continuously reacting to its 
surroundings. This is why the controller behaves robustly in the simulator even though it uses a relatively 
simple algorithm. 
Figure 1. Flowchart of the obstacle avoidance logic used by the TurtleBot3 controller. 
5. Results and Testing. 
The completed controller was tested in Gazebo using the supplied obstacle course. During testing the 
robot moved forward successfully when no obstacle was present in the front region. When the front region 
became too narrow, the robot stopped and selected a turn direction based on the wider side clearance. 
The screenshots in Appendix B show different stages of the simulation. The robot was able to navigate 
around the perimeter and between internal obstacles for an extended period without directly colliding with 
them. This indicates that the chosen front threshold and turning logic were sufficient for the test 
environment. 
Testing also confirmed that allowing turns in both directions improves performance compared with a single
direction turn strategy. By checking which side has greater free space, the robot avoids some unnecessary 
rotations and is less likely to remain trapped near walls. 
6. Discussion. 
One of the main strengths of the final implementation is that it is simple, readable and directly linked to the 
sensor information being received. The use of averaged LiDAR sectors makes the control logic easy to 
interpret: the front sector decides whether the robot should continue straight, while the side sectors decide 
which direction is safer for turning. 
The selected threshold of 0.7 m gives the robot enough margin to stop and rotate before making contact 
with an obstacle. If this threshold were reduced too much, the robot would wait too long before reacting, 
which could lead to collisions. If it were increased too much, the robot would turn too often and appear 
overly cautious. Therefore, the threshold acts as a tuning parameter that balances safety and efficiency. 
Similarly, the turn rate of ±0.8 rad/s is fast enough to reorient the robot without causing unstable motion in 
the simulator. Combined with a forward speed of 0.2 m/s, this produces behaviour that is smooth enough 
to observe and effective enough to satisfy the task requirements. 
A limitation of the controller is that it is purely reactive. It does not remember previously visited areas and 
does not plan a global route. In complex layouts this could lead to repeated loops or inefficient coverage. 
However, for the assignment brief the objective is reliable obstacle avoidance rather than optimal path 
planning, so the chosen method is appropriate. 
Possible improvements would include using more sectors, adding hysteresis to reduce oscillation, or 
integrating mapping and localisation so that the robot could make higher-level navigation decisions. 
7. Conclusion. 
In conclusion, the TurtleBot3 (Burger) controller successfully achieves autonomous obstacle avoidance in 
Gazebo using ROS2. The code reads LiDAR data, separates the scan into front and side regions, checks 
whether the front distance is below 0.7 m, and then commands the robot either to move forward at 0.2 m/s 
or rotate at ±0.8 rad/s depending on which side is more open. 
The final implementation satisfies the key assignment requirements because the robot moves, stops when 
necessary, turns intelligently in both directions, and can continue navigating the environment for a 
prolonged period without crashing.