#include "rclcpp/rclcpp.hpp"
#include "Turtlebot3Driver.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Turtlebot3Driver>();
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

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

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
