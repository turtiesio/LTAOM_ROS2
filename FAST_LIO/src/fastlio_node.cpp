/*
 * FAST-LIO2 ROS2 Node Entry Point
 * Ported from ROS1 to ROS2
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>

// Forward declaration - implemented in laserMapping.cpp
int mainLIOFunction(rclcpp::Node::SharedPtr node);

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("fastlio_mapping");

    RCLCPP_INFO(node->get_logger(), "FAST-LIO2 ROS2 Node Starting...");

    int result = mainLIOFunction(node);

    rclcpp::shutdown();
    return result;
}
