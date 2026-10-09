#include "Turtlebot3Driver.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

Turtlebot3Driver::Turtlebot3Driver()
: Node("turtlebot3_driver"),
  isObstacleDetected(false),
  frontDistance(std::numeric_limits<double>::infinity()),
  leftDistance(std::numeric_limits<double>::infinity()),
  rightDistance(std::numeric_limits<double>::infinity())
{
    commandPub = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

    auto qos_profile = rclcpp::SensorDataQoS();
    laserSub = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        qos_profile,
        std::bind(&Turtlebot3Driver::scanCallback, this, std::placeholders::_1)
    );
}

void Turtlebot3Driver::moveForward() {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";

    msg->twist.linear.x = FORWARD_SPEED;
    msg->twist.angular.z = 0.0;

    commandPub->publish(std::move(msg));
}

void Turtlebot3Driver::stopMoving() {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";

    msg->twist.linear.x = 0.0;
    msg->twist.angular.z = 0.0;

    commandPub->publish(std::move(msg));
}

void Turtlebot3Driver::turn(double angular) {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";

    msg->twist.linear.x = 0.0;
    msg->twist.angular.z = angular;

    commandPub->publish(std::move(msg));
}

bool Turtlebot3Driver::obstacleDetected() const {
    return isObstacleDetected;
}

double Turtlebot3Driver::getFrontDistance() const {
    return frontDistance;
}

double Turtlebot3Driver::getLeftDistance() const {
    return leftDistance;
}

double Turtlebot3Driver::getRightDistance() const {
    return rightDistance;
}

double Turtlebot3Driver::averageRange(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    double start_deg,
    double end_deg)
{
    const int n = static_cast<int>(scan->ranges.size());

    if (n == 0) {
        return std::numeric_limits<double>::infinity();
    }

    const double angle_min_deg = scan->angle_min * 180.0 / M_PI;
    const double angle_inc_deg = scan->angle_increment * 180.0 / M_PI;

    double sum = 0.0;
    int count = 0;

    for (int i = 0; i < n; i++) {
        double angle_deg = angle_min_deg + i * angle_inc_deg;

        if (angle_deg >= start_deg && angle_deg <= end_deg) {
            double r = scan->ranges[i];

            if (std::isfinite(r) && r >= scan->range_min && r <= scan->range_max) {
                sum += r;
                count++;
            }
        }
    }

    if (count == 0) {
        return std::numeric_limits<double>::infinity();
    }

    return sum / static_cast<double>(count);
}

void Turtlebot3Driver::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    const double MIN_DIST_FROM_OBSTACLE = 0.7;

    const int n = static_cast<int>(scan->ranges.size());

    if (n <= 0 || scan->angle_increment == 0.0) {
        isObstacleDetected = false;
        frontDistance = std::numeric_limits<double>::infinity();
        leftDistance = std::numeric_limits<double>::infinity();
        rightDistance = std::numeric_limits<double>::infinity();
        return;
    }

    double front_left = averageRange(scan, -35.0, 0.0);
    double front_right = averageRange(scan, 0.0, 35.0);
    frontDistance = std::min(front_left, front_right);

    leftDistance = averageRange(scan, 35.0, 135.0);
    rightDistance = averageRange(scan, -135.0, -35.0);

    isObstacleDetected = (frontDistance < MIN_DIST_FROM_OBSTACLE);
}
