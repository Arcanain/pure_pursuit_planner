// Directory: pure_pursuit_planner/src/pure_pursuit_component.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include <cmath>
#include <limits>
#include <iostream>

namespace pure_pursuit_planner {

PurePursuitComponent::PurePursuitComponent(const PurePursuitConfig& config)
: cfg_(config) {}

void PurePursuitComponent::setPath(const std::vector<double>& cx,
                                    const std::vector<double>& cy,
                                    const std::vector<double>& cyaw,
                                    const std::vector<double>& ck) {
    cx_ = cx;
    cy_ = cy;
    cyaw_ = cyaw;
    ck_ = ck;
}

void PurePursuitComponent::setPose(const Pose2D& pose, double velocity) {
    current_pose_ = pose;
    current_velocity_ = velocity;
}

void PurePursuitComponent::setObstacle(double x, double y, bool detected) {
    obstacle_x_ = x;
    obstacle_y_ = y;
    obstacle_detected_ = detected;
}

std::pair<double, double> PurePursuitComponent::computeVelocity() {
    
    auto [ind, Lf] = searchTargetIndex();
    std::cout << "ind: " << ind << std::endl;
    std::cout << "Lf: " << Lf << std::endl;
    std::cout << "cx: " << cx_.size() << std::endl;
    if (ind >= cx_.size()) {
        goal_reached_ = true;
        return {0.0, 0.0};
    }

    targetIndex_ = ind;

    double tx = cx_[targetIndex_];
    double ty = cy_[targetIndex_];
    double target_yaw = cyaw_[targetIndex_];
    double target_curvature = ck_[targetIndex_];

    double dx = tx - current_pose_.x;
    double dy = ty - current_pose_.y;

    double alpha = std::atan2(dy, dx) - current_pose_.yaw;

    double curvature = std::max(cfg_.minCurvature, std::min(std::abs(target_curvature), cfg_.maxCurvature));
    curvature = curvature / cfg_.maxCurvature;
    double v = (cfg_.maxCurvature- cfg_.minCurvature) * pow(sin(acos(std::cbrt(curvature))), 3) + cfg_.minVelocity; //[m/s]
    //v = std::clamp(v, cfg_.minVelocity, cfg_.maxVelocity);
    std::cout << "v: " << v << std::endl;

    double w = v * std::sin(alpha) / Lf;
    //w = std::clamp(w, -cfg_.maxAngularVelocity, cfg_.maxAngularVelocity);

    return {v, w};
}

bool PurePursuitComponent::isGoalReached() const {
    return goal_reached_;
}

std::pair<int, double> PurePursuitComponent::searchTargetIndex() {
    double Lf = cfg_.k * current_velocity_ + cfg_.Lfc;
    std::cout << "Lfc: " << cfg_.Lfc << std::endl;
    double min_dist = std::numeric_limits<double>::max();
    int nearest_index = 0;

    for (size_t i = 0; i < cx_.size(); ++i) {
        double d = calcDistance(current_pose_.x, current_pose_.y, cx_[i], cy_[i]);
        if (d < min_dist) {
            min_dist = d;
            nearest_index = static_cast<int>(i);
        }
    }

    double dist = 0.0;
    for (size_t i = nearest_index; i < cx_.size(); ++i) {
        dist = calcDistance(current_pose_.x, current_pose_.y, cx_[i], cy_[i]);
        if (dist > Lf) {
            return {static_cast<int>(i), Lf};
        }
    }
    return {static_cast<int>(cx_.size() - 1), Lf};
    
}

double PurePursuitComponent::calcDistance(double x1, double y1, double x2, double y2) const {
    return std::hypot(x2 - x1, y2 - y1);
}

} // namespace pure_pursuit_planner
