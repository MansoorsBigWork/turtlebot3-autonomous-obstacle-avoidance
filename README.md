# TurtleBot3 Autonomous Obstacle Avoidance

A reactive obstacle-avoidance controller for the **TurtleBot3 Burger**, written in C++ as a ROS 2 node and tested in Gazebo.

The robot drives forward until its LiDAR sees something closer than 0.7 m ahead. It then stops and rotates towards whichever side has more open space, and carries on. There is no map and no path planning: it just senses, decides and acts ten times a second.

> Built for ELEC230 Assignment 3 (University of Liverpool). The full write-up is in [`Main/README.md`](Main/README.md).

## How it works

```
        ┌──────────────┐   /scan (LaserScan)   ┌─────────────────────┐   /cmd_vel (TwistStamped)
Gazebo ─┤ LiDAR sensor ├──────────────────────►│  turtlebot3_driver  ├──────────────────────────► robot
        └──────────────┘                       └─────────────────────┘
```

Each scan is split into angular sectors, and the valid readings in each sector are averaged:

| Sector       | Angles          | Used for                                    |
|--------------|-----------------|---------------------------------------------|
| Front-left   | −35° to 0°      | front distance = min(front-left, front-right) |
| Front-right  | 0° to +35°      |                                             |
| Left         | +35° to +135°   | choosing the turn direction                 |
| Right        | −135° to −35°   |                                             |

Readings that are infinite or outside the sensor's range are ignored. A sector with no valid readings counts as clear.

The control loop runs at 10 Hz:

```cpp
if (!node->obstacleDetected()) {          // front distance >= 0.7 m
    node->moveForward();                  // 0.2 m/s
} else {
    node->stopMoving();
    if (node->getLeftDistance() > node->getRightDistance())
        node->turn(0.8);                  // turn left  (rad/s)
    else
        node->turn(-0.8);                 // turn right
}
```

### Tuning parameters

| Parameter          | Value      | Where                                         |
|--------------------|------------|-----------------------------------------------|
| Forward speed      | 0.2 m/s    | `FORWARD_SPEED` in `Turtlebot3Driver.hpp`     |
| Turn speed         | ±0.8 rad/s | `main.cpp`                                    |
| Obstacle threshold | 0.7 m      | `MIN_DIST_FROM_OBSTACLE` in `Turtlebot3Driver.cpp` |
| Loop rate          | 10 Hz      | `rclcpp::Rate loop_rate(10)` in `main.cpp`    |

## Project layout

```
Main/                         ROS 2 package "assignment3"
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── elec230.launch.py     starts the TurtleBot3 Gazebo world and the driver node
└── src/
    ├── main.cpp              10 Hz decision loop
    ├── Turtlebot3Driver.hpp  node class declaration
    └── Turtlebot3Driver.cpp  LiDAR processing and velocity publishing
```

## Requirements

- ROS 2 with Gazebo (gz sim). The node publishes `TwistStamped` on `cmd_vel`, which matches the TurtleBot3 packages for ROS 2 Jazzy.
- The `turtlebot3_gazebo` package (from `turtlebot3_simulations`)
- `TURTLEBOT3_MODEL=burger`

## Build and run

```bash
# Copy the package into a colcon workspace
mkdir -p ~/ros2_ws/src
cp -r Main ~/ros2_ws/src/assignment3

# Build
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select assignment3
source install/setup.bash

# Launch Gazebo and the driver
export TURTLEBOT3_MODEL=burger
ros2 launch assignment3 elec230.launch.py
```

The launch file opens the standard `turtlebot3_world`, starts `turtlebot3_driver` with `use_sim_time:=true`, and kills any leftover Gazebo (`ruby`) processes when it shuts down.

## Limitations and ideas

- **Purely reactive.** The robot doesn't remember where it has been, so coverage can be uneven and it can loop in some layouts.
- **No hysteresis.** Near the 0.7 m threshold the robot can switch back and forth between driving and turning.
- **Possible improvements:** use more, finer sectors, add a hysteresis band, turn while still moving forward, or add SLAM and Nav2 for higher-level navigation.
