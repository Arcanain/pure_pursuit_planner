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

private:
    void updateControl();
    std::pair<double, double> purePursuitControl(int& pind);
    std::pair<int, double> searchTargetIndex();
    double calcDistance(double point_x, double point_y) const;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void publishCmd(double v, double w);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::TimerBase::SharedPtr timer;
    double x, y, yaw, v, w;
    int target_ind;
    int oldNearestPointIndex;
    double target_vel;
    double current_vel;

    // check goal dist
    double goal_threshold = 0.1; //[m]

    // pure pursuit parameter
    const double k = 0.1; // look forward gain
    //const double Lfc = 2.0; // [m] look-ahead distance
    const double Lfc = 0.25; // [m] look-ahead distance
    const double Kp = 1.0; // speed proportional gain
    const double dt = 0.1; // [s] time tick

    // cauvature parameter
    double minCurvature = 0.0;
    double maxCurvature = 3.0;
    double minVelocity = 0.1;
    double maxVelocity = 0.3;
};

#endif // PURE_PURSUIT_PLANNER_COMPONENT_HPP
