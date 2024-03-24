#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode()
: Node("pure_pursuit_planner") {
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("target_path", 10);
    // odometoryのSubscribe
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));

    timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                    std::bind(&PurePursuitNode::updateControl, this));
    std::iota(cx.begin(), cx.end(), 0.0);
    std::transform(cx.begin(), cx.end(), cy.begin(), [](double ix) { return std::sin(ix / 5.0) * ix / 2.0; });
    std::tie(target_ind, std::ignore) = searchTargetIndex();
}

void PurePursuitNode::updateControl() {
    if (T < 0 || target_ind >= static_cast<int>(cx.size()) - 1) {
        rclcpp::shutdown();
        return;
    }

    auto [v, w] = purePursuitControl(target_ind);
    T -= dt;

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = v;
    cmd_vel_msg.angular.z = w;
    cmd_vel_pub->publish(cmd_vel_msg);

    publishPath();
}

std::pair<double, double> PurePursuitNode::purePursuitControl(int& pind) {
    auto [ind, Lf] = searchTargetIndex();

    if (pind >= ind) {
        ind = pind;
    }

    double tx, ty;
    if (ind < static_cast<int>(cx.size())) {
        tx =cx[ind];
        ty = cy[ind];
    } else {
        tx = cx.back();
        ty = cy.back();
        ind = static_cast<int>(cx.size()) - 1;
    }

    double alpha = std::atan2(ty - y, tx - x) - yaw;
    double v = target_speed;
    double w = v * std::tan(alpha) / Lf;

    pind = ind;
    return { v, w };
}

void PurePursuitNode::publishPath() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (size_t i = 0; i < cx.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = path_msg.header.stamp;
        pose.header.frame_id = path_msg.header.frame_id;
        pose.pose.position.x = cx[i];
        pose.pose.position.y = cy[i];
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub->publish(path_msg);
}

std::pair<int, double> PurePursuitNode::searchTargetIndex() {
    if (oldNearestPointIndex == -1) {
        std::vector<double> dx(cx.size()), dy(cy.size());
        for (size_t i = 0; i < cx.size(); ++i) {
            dx[i] = x - cx[i];
            dy[i] = y - cy[i];
        }
        std::vector<double> d(dx.size());
        std::transform(dx.begin(), dx.end(), dy.begin(), d.begin(), [](double dx, double dy) { return std::hypot(dx, dy); });
        auto it = std::min_element(d.begin(), d.end());
        oldNearestPointIndex = std::distance(d.begin(), it);
    } else {
        while (true) {
            double distanceThisIndex = calcDistance(cx[oldNearestPointIndex], cy[oldNearestPointIndex]);
            double distanceNextIndex = calcDistance(cx[oldNearestPointIndex + 1], cy[oldNearestPointIndex + 1]);
            if (distanceThisIndex < distanceNextIndex) {
                break;
            }
            oldNearestPointIndex++;
            if (oldNearestPointIndex >= static_cast<int>(cx.size()) - 1) {
                break;
            }
        }
    }

    double Lf = k * v + Lfc;

    int ind = oldNearestPointIndex;
    while (Lf > calcDistance(cx[ind], cy[ind])) {
        if (ind + 1 >= static_cast<int>(cx.size())) {
            break;
        }
        ind++;
    }

    return { ind, Lf };
}

double PurePursuitNode::calcDistance(double point_x, double point_y) const {
    double dx = x - point_x;
    double dy = y - point_y;
    return std::hypot(dx, dy);
}

void PurePursuitNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // オドメトリからx, y, thetaを取得
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    yaw = yaw_tmp;
}
