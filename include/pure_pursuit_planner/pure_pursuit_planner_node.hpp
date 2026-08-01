// Directory: pure_pursuit_planner/include/pure_pursuit_planner/pure_pursuit_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

namespace pure_pursuit_planner {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PurePursuitNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit PurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    // Lifecycle transitions
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
    // 受信して内部状態を更新する（Active/Inactive を通じて常時動かす）
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // 自走して計算・発信する（Active のときだけ動かす）
    void timerCallback();

    // ① deactivate 時に明示的な停止指令を送る
    void publishZeroVelocity();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PurePursuitComponent planner_;

    std::vector<double> cx_, cy_, cyaw_, ck_;
    bool path_received_ = false;
    bool pose_received_ = false;
    bool path_subscribe_flag = false;

    double current_vx_ = 0.0;
    Pose2D current_pose_;

    // パラメータ
    PurePursuitConfig config_;

    void declareAndGetParameters();
};

} // namespace pure_pursuit_planner
