// Directory: pure_pursuit_planner/src/pure_pursuit_planner.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pure_pursuit_planner::PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
