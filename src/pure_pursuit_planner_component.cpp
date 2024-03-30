#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode()
: Node("pure_pursuit_planner") {
    // Parameter setting
    target_ind = 0;
    oldNearestPointIndex = -1;
    //target_vel = 10.0 / 3.6;
    target_vel = 0.25;

    // Publisher
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "tgt_path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
    
    // Timer callback
    timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                    std::bind(&PurePursuitNode::updateControl, this));
}

void PurePursuitNode::updateControl() {
    // if (T < 0 || target_ind >= static_cast<int>(cx.size()) - 1) {
    //     rclcpp::shutdown();
    //     return;
    // }

    auto [v, w] = purePursuitControl(target_ind);

    publishCmd(v, w);
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
    double v = target_vel;
    double w = v * std::tan(alpha) / Lf;

    pind = ind;
    return { v, w };
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

void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!path_subscribe_flag) {
        // 受け取ったパスメッセージから座標を抽出
        for (const auto& pose : msg->poses) {
            cx.push_back(pose.pose.position.x);
            cy.push_back(pose.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Received path point: (%f, %f)", pose.pose.position.x, pose.pose.position.y);
        }

        // 最新のパスに基づいて目標インデックスを更新
        std::tie(target_ind, std::ignore) = searchTargetIndex();
    }

    path_subscribe_flag = true;
}

void PurePursuitNode::publishCmd(double v, double w)
{
    geometry_msgs::msg::Twist cmd_vel_msg;

    double goal_x = cx[cx.size() - 1];
    double goal_y = cy[cy.size() - 1];
    double goal_dist = std::abs(std::sqrt(std::pow((goal_x - x), 2.0) + std::pow((goal_y - y), 2.0)));

    // goal judgement
    if (goal_dist < goal_threshold) {
        std::cout << "Goal!" << std::endl;

        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
    } else {
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = w;
    }

    cmd_vel_pub->publish(cmd_vel_msg);
}
