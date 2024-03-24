#include "rclcpp/rclcpp.hpp"
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
