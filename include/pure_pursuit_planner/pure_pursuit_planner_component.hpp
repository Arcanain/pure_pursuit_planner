#ifndef PURE_PURSUIT_PLANNER_COMPONENT_HPP
#define PURE_PURSUIT_PLANNER_COMPONENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <math.h>

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode();
    std::vector<double> cx;
    std::vector<double> cy;
    std::vector<double> cyaw;
    std::vector<double> ck;

    // path subscribe flag
    bool path_subscribe_flag = false;
    bool odom_subscribe_flag = false;
    bool obstacle_detected = false;

    bool avoidance_flag = false;

private:
    void updateControl();
    std::pair<double, double> purePursuitControl(int& pind);
    std::pair<int, double> searchTargetIndex();
    std::pair<double, std::pair<double, double>> calcClosestPointOnPath();
    double calcDistance(double point_x, double point_y) const;
    std::pair<double, double> calcAcceleration(double current_vel, rclcpp::Time now_time) ;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void obstacle_detected_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void publishCmd(double v, double w);
    void visualizeTargetPoint(double target_lookahed_x, double target_lookahed_y);
    void visualizeTargetCircle(double target_lookahed_x, double target_lookahed_y);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_range_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_range_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lidar_range_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_status_pub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time current_time;

    double x, y, yaw, v, w, obstacle_x, obstacle_y;
    int target_ind;
    int oldNearestPointIndex;
    double target_vel;
    double current_vel;

    double temp_target_x = 0.0;
    double temp_target_y = 0.0;

    double pre_min_distance = 0.0;
    double diff_min_dist;
    double init_x, init_y;

    double previous_vel = 0.0;
    double max_acceleration = 0.08;
    rclcpp::Time previous_time = this->get_clock()->now();


    //obstacle parameter
    double obstacle_th = 0.5;

    // check goal dist
    double goal_threshold = 0.4; //[m]

    // pure pursuit parameter
    const double k = 0.5; // look forward gain
    //const double Lfc = 2.0; // [m] look-ahead distance
    const double Lfc = 0.8; // [m] look-ahead distance
    const double Kp = 1.0; // speed proportional gain
    const double dt = 0.1; // [s] time tick

    // cauvature parameter
    double minCurvature = 0.0;
    double maxCurvature = 3.0;
    double minVelocity = 0.4;
    double maxVelocity = 0.7;

    double max_angular_velocity = 1.3;
};

#endif // PURE_PURSUIT_PLANNER_COMPONENT_HPP
