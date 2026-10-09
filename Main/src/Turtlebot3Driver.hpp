#ifndef TURTLEBOT3_DRIVER_HPP_
#define TURTLEBOT3_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class Turtlebot3Driver : public rclcpp::Node {
public:
    Turtlebot3Driver();

    void moveForward();
    void stopMoving();
    void turn(double angular);

    bool obstacleDetected() const;
    double getFrontDistance() const;
    double getLeftDistance() const;
    double getRightDistance() const;

private:
    static constexpr double FORWARD_SPEED = 0.2;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr commandPub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;

    bool isObstacleDetected;
    double frontDistance;
    double leftDistance;
    double rightDistance;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    double averageRange(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                        double start_deg,
                        double end_deg);
};

#endif
