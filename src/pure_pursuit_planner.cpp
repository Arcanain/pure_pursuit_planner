// Directory: pure_pursuit_planner/src/pure_pursuit_planner.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // LifecycleNode は base interface を介して spin する。
    // configure / activate などの遷移はライフサイクルマネージャ等の外部から行う。
    auto node = std::make_shared<pure_pursuit_planner::PurePursuitNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
