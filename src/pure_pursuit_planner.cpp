#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

const double k = 0.1; // look forward gain
const double Lfc = 2.0; // [m] look-ahead distance
const double Kp = 1.0; // speed proportional gain
const double dt = 0.1; // [s] time tick

class RobotState {
public:
    double x, y, yaw, v, w;

    RobotState(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0, double w = 0.0)
        : x(x), y(y), yaw(yaw), v(v), w(w) {}

    void update(double v, double w) {
        x += v * std::cos(yaw) * dt;
        y += v * std::sin(yaw) * dt;
        yaw += w * dt;
        this->v = v;
        this->w = w;
    }

    double calcDistance(double point_x, double point_y) const {
        double dx = x - point_x;
        double dy = y - point_y;
        return std::hypot(dx, dy);
    }

};

class TargetCourse {
public:
    std::vector<double> cx, cy;
    int oldNearestPointIndex = -1;

    TargetCourse(const std::vector<double>& cx, const std::vector<double>& cy)
        : cx(cx), cy(cy) {}

    std::pair<int, double> searchTargetIndex(const RobotState& state) {
        if (oldNearestPointIndex == -1) {
            std::vector<double> dx(cx.size()), dy(cy.size());
            for (size_t i = 0; i < cx.size(); ++i) {
                dx[i] = state.x - cx[i];
                dy[i] = state.y - cy[i];
            }
            std::vector<double> d(dx.size());
            std::transform(dx.begin(), dx.end(), dy.begin(), d.begin(), [](double dx, double dy) { return std::hypot(dx, dy); });
            auto it = std::min_element(d.begin(), d.end());
            oldNearestPointIndex = std::distance(d.begin(), it);
        } else {
            while (true) {
                double distanceThisIndex = state.calcDistance(cx[oldNearestPointIndex], cy[oldNearestPointIndex]);
                double distanceNextIndex = state.calcDistance(cx[oldNearestPointIndex + 1], cy[oldNearestPointIndex + 1]);
                if (distanceThisIndex < distanceNextIndex) {
                    break;
                }
                oldNearestPointIndex++;
                if (oldNearestPointIndex >= static_cast<int>(cx.size()) - 1) {
                    break;
                }
            }
        }

        double Lf = k * state.v + Lfc;

        int ind = oldNearestPointIndex;
        while (Lf > state.calcDistance(cx[ind], cy[ind])) {
            if (ind + 1 >= static_cast<int>(cx.size())) {
                break;
            }
            ind++;
        }

        return { ind, Lf };
    }
};

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode()
    : Node("pure_pursuit"), target_ind(0), T(100.0)
    {
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        path_pub = this->create_publisher<nav_msgs::msg::Path>("target_path", 10);

        timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                        std::bind(&PurePursuitNode::updateControl, this));
        std::iota(cx.begin(), cx.end(), 0.0);
        std::transform(cx.begin(), cx.end(), cy.begin(), [](double ix) { return std::sin(ix / 5.0) * ix / 2.0; });
        target_course = std::make_unique<TargetCourse>(cx, cy);
        std::tie(target_ind, std::ignore) = target_course->searchTargetIndex(state);
    }

private:
    void updateControl() {
        if (T < 0 || target_ind >= static_cast<int>(cx.size()) - 1) {
            rclcpp::shutdown();
            return;
        }

        auto [v, w] = purePursuitControl(state, *target_course, target_ind);
        state.update(v, w);
        T -= dt;

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = w;
        cmd_vel_pub->publish(cmd_vel_msg);

        publishPath();
    }

    std::pair<double, double> purePursuitControl(const RobotState& state, TargetCourse& trajectory, int& pind) {
        auto [ind, Lf] = trajectory.searchTargetIndex(state);

        if (pind >= ind) {
            ind = pind;
        }

        double tx, ty;
        if (ind < static_cast<int>(trajectory.cx.size())) {
            tx = trajectory.cx[ind];
            ty = trajectory.cy[ind];
        } else {
            tx = trajectory.cx.back();
            ty = trajectory.cy.back();
            ind = static_cast<int>(trajectory.cx.size()) - 1;
        }

        double alpha = std::atan2(ty - state.y, tx - state.x) - state.yaw;
        double v = target_speed;
        double w = v * std::tan(alpha) / Lf;

        pind = ind;
        return { v, w };
    }

    void publishPath() {
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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::TimerBase::SharedPtr timer;
    RobotState state;
    std::vector<double> cx = std::vector<double>(100);
    std::vector<double> cy = std::vector<double>(100);
    std::unique_ptr<TargetCourse> target_course;
    int target_ind;
    double T;
    const double target_speed = 10.0 / 3.6; // [m/s]
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
