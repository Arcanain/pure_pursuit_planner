// Directory: pure_pursuit_planner/src/pure_pursuit_node.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"


namespace pure_pursuit_planner {

PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
: Node("pure_pursuit_node", options), planner_(config_) {

    declareAndGetParameters();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "tgt_path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PurePursuitNode::timerCallback, this));
    planner_ = PurePursuitComponent(config_);  // 値が入ったconfig_で再初期化
    
}

void PurePursuitNode::declareAndGetParameters() {
    config_.k = this->declare_parameter("k", 0.5);
    config_.Lfc = this->declare_parameter("Lfc", 0.8);
    config_.Kp = this->declare_parameter("Kp", 1.0);
    config_.dt = this->declare_parameter("dt", 0.1);
    config_.goal_threshold = this->declare_parameter("goal_threshold", 0.4);
    config_.max_acceleration = this->declare_parameter("max_acceleration", 0.08);
    config_.minCurvature = this->declare_parameter("minCurvature", 0.0);
    config_.maxCurvature = this->declare_parameter("maxCurvature", 3.0);
    config_.minVelocity = this->declare_parameter("minVelocity", 0.4);
    config_.maxVelocity = this->declare_parameter("maxVelocity", 0.7);
    config_.maxAngularVelocity = this->declare_parameter("maxAngularVelocity", 1.3);
    config_.obstacle_th = this->declare_parameter("obstacle_th", 0.5);
}

void PurePursuitNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "start Received path point");
    //cx_.clear(); cy_.clear(); cyaw_.clear(); ck_.clear();
    if (!path_subscribe_flag) {
        // 受け取ったパスメッセージから座標を抽出
        for (const auto& pose : msg->poses) {
            cx_.push_back(pose.pose.position.x);
            cy_.push_back(pose.pose.position.y);
            ck_.push_back(pose.pose.position.z);

            tf2::Quaternion quat;
            tf2::fromMsg(pose.pose.orientation, quat);
            tf2::Matrix3x3 mat(quat);
            double roll_rev, pitch_rev, yaw_rev;
            mat.getRPY(roll_rev, pitch_rev, yaw_rev);
            cyaw_.push_back(yaw_rev);

            //RCLCPP_INFO(this->get_logger(), "Received path point: (%f, %f)", pose.pose.position.x, pose.pose.position.y);
        }
        
        path_received_ = true;
        path_subscribe_flag = true;
    }
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    //current_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    current_vx_ = msg->twist.twist.linear.x;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    current_pose_.yaw = yaw_tmp;

    pose_received_ = true;
}

void PurePursuitNode::timerCallback() {
    //RCLCPP_INFO(this->get_logger(), "Timer triggered. path_received_: %d, pose_received_: %d", path_received_, pose_received_);

    if (!path_received_ || !pose_received_) return;

    auto [v, w] = planner_.computeVelocity(cx_, cy_, cyaw_, ck_, current_pose_, current_vx_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;

    cmd_vel_pub_->publish(cmd_vel);
}

}  // namespace pure_pursuit_planner
