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
: rclcpp_lifecycle::LifecycleNode("pure_pursuit_node", options), planner_(config_) {

    // パラメータはコンストラクタで宣言し、planner_ を確定した config_ で初期化する。
    // I/O（sub/pub/timer）の生成は on_configure 以降で行う。
    declareAndGetParameters();
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

// =====================================================================
// Lifecycle transitions
// =====================================================================

CallbackReturn PurePursuitNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "on_configure: setting up subscriptions / publisher / timer");

    // 受信系（状態更新）は configure で生成し、Active/Inactive を通じて生かしておく
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "tgt_path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    // LifecyclePublisher: Inactive 時は publish が自動的に破棄される（多重安全）
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    lookahead_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("pp_lookahead_marker", 10);

    // 能動駆動のタイマーは生成するが、activate されるまでは止めておく
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PurePursuitNode::timerCallback, this));
    timer_->cancel();

    return CallbackReturn::SUCCESS;
}

CallbackReturn PurePursuitNode::on_activate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "on_activate: re-acquire nearest point and start control loop");

    // ② 再開時は追従進捗をリセットし、現在位置から最近傍点を取り直す
    planner_.oldNearestPointIndex = -1;

    cmd_vel_pub_->on_activate();
    lookahead_marker_pub_->on_activate();
    timer_->reset();  // 能動駆動（computeVelocity + publish）を開始

    return CallbackReturn::SUCCESS;
}

CallbackReturn PurePursuitNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "on_deactivate: stop control loop and command zero velocity");

    // まず能動駆動を止める（ステートフルな computeVelocity も一緒に止まる）
    timer_->cancel();

    // ① publisher がまだ active のうちに明示的な停止指令を 1 回送る
    publishZeroVelocity();

    cmd_vel_pub_->on_deactivate();
    lookahead_marker_pub_->on_deactivate();

    return CallbackReturn::SUCCESS;
}

CallbackReturn PurePursuitNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "on_cleanup: releasing I/O and resetting state");

    timer_.reset();
    cmd_vel_pub_.reset();
    lookahead_marker_pub_.reset();
    path_sub_.reset();
    odom_sub_.reset();

    // 再 configure に備えて内部状態を初期化する
    cx_.clear(); cy_.clear(); cyaw_.clear(); ck_.clear();
    path_received_ = false;
    pose_received_ = false;
    path_subscribe_flag = false;
    planner_ = PurePursuitComponent(config_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn PurePursuitNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "on_shutdown");

    if (timer_) {
        timer_->cancel();
    }
    // Active から直接 shutdown された場合に備え、停止指令を送ってから解放する
    if (cmd_vel_pub_ && cmd_vel_pub_->is_activated()) {
        publishZeroVelocity();
        cmd_vel_pub_->on_deactivate();
    }

    timer_.reset();
    cmd_vel_pub_.reset();
    path_sub_.reset();
    odom_sub_.reset();

    return CallbackReturn::SUCCESS;
}

// =====================================================================
// I/O callbacks
// =====================================================================

void PurePursuitNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
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
        }

        path_received_ = true;
        path_subscribe_flag = true;
    }
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
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
    if (!path_received_ || !pose_received_) return;

    auto cmd_velocity = planner_.computeVelocity(cx_, cy_, cyaw_, ck_, current_pose_, current_vx_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = cmd_velocity[0];
    cmd_vel.angular.z = cmd_velocity[1];
    cmd_vel_pub_->publish(cmd_vel);

    // 前方注視点を緑のSphereマーカーで可視化
    int idx = planner_.getTargetIndex();
    if (idx >= 0 && idx < static_cast<int>(cx_.size())) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = now();
        marker.ns = "pp_lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = cx_[idx];
        marker.pose.position.y = cy_[idx];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0;
        lookahead_marker_pub_->publish(marker);
    }
}

void PurePursuitNode::publishZeroVelocity() {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);
}

}  // namespace pure_pursuit_planner
