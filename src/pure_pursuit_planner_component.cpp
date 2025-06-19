// Directory: pure_pursuit_planner/src/pure_pursuit_component.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>

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
    std::cout << "odom_sub_flag: " << odom_sub_flag << std::endl;
    if (!odom_sub_flag){
        odom_sub_flag = true;
        oldNearestPointIndex = -1;
    }
}

void PurePursuitComponent::setPose(const Pose2D& pose, double velocity) {
    current_pose_ = pose;
    current_velocity_ = velocity;
}

std::pair<double, double> PurePursuitComponent::computeVelocity(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    const Pose2D& pose, 
    double velocity
) 
    {
    setPath(cx, cy, cyaw, ck);
    setPose(pose, velocity);
    std::cout << "odom_sub_flag: " << odom_sub_flag << std::endl;
    auto [ind, Lf] = searchTargetIndex();
    std::cout << "ind: " << ind << std::endl;
    std::cout << "Lf: " << Lf << std::endl;
    std::cout << "cx: " << cx_.size() << std::endl;

    targetIndex_ = ind;

    double tx = cx_[targetIndex_];
    double ty = cy_[targetIndex_];
    double target_yaw = cyaw_[targetIndex_];
    double target_curvature = ck_[targetIndex_];
    std::cout << "target_curvature: " << std::abs(target_curvature) << std::endl;
    double dx = tx - current_pose_.x;
    double dy = ty - current_pose_.y;

    double alpha = std::atan2(dy, dx) - current_pose_.yaw;

    double curvature = std::max(cfg_.minCurvature, std::min(std::abs(target_curvature), cfg_.maxCurvature));
    std::cout << "curvature: " << std::abs(curvature) << std::endl;
    curvature = curvature / cfg_.maxCurvature;

    double v = (cfg_.maxCurvature- cfg_.minCurvature) * pow(sin(acos(std::cbrt(curvature))), 3) + cfg_.minVelocity; //[m/s]
    v = std::clamp(v, cfg_.minVelocity, cfg_.maxVelocity);
    std::cout << "v: " << v << std::endl;

    double w = v * std::sin(alpha) / Lf;
    //w = std::clamp(w, -cfg_.maxAngularVelocity, cfg_.maxAngularVelocity);

    std::tie(v, w) = isGoalReached(v, w);


    return {v, w};
}

std::pair<double, double> PurePursuitComponent::isGoalReached(double v, double w) const {
    size_t last_index = cx_.size() - 1;
    double goal_distance = calcDistance(current_pose_.x, current_pose_.y, cx_[last_index], cy_[last_index]);

    if (goal_distance < 0.1){
        v = 0.0;
        w = 0.0;
    }
    return {v, w};
}

double PurePursuitComponent::calcLf(double k, double current_velocity, double Lfc) const {
    return k * current_velocity + Lfc;
}
int PurePursuitComponent::calcFirstNearestPointIndex() const {
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for (size_t i = 0; i < cx_.size(); i++) {
        double distance = calcDistance(current_pose_.x, current_pose_.y, cx_[i], cy_[i]);
        if (distance < min_distance) {
            min_distance = distance;
            min_index = i;
        }
    }
    return min_index;
}

int PurePursuitComponent::calcOldNearestPointIndex() const {
    bool count_flag = false;
    int count = 0, min_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    
    std::vector<double> min_distance_list; 
    std::vector<int> min_distance_idx_list; 
    min_distance_list.clear();
    min_distance_idx_list.clear();
    int search_start = std::max(0, oldNearestPointIndex - 20);
    for (int i = search_start; i < static_cast<int>(cx_.size()) - 1; i++) {
        double distanceThisIndex = calcDistance(current_pose_.x, current_pose_.y, cx_[i],     cy_[i]);
        double distanceNextIndex = calcDistance(current_pose_.x, current_pose_.y, cx_[i + 1], cy_[i + 1]);
        if (distanceThisIndex < distanceNextIndex) {
            count_flag = true;
        }
        if (count_flag){
            count ++;
        }
        if (distanceThisIndex < min_distance) {
            min_distance = distanceThisIndex;
            //min_index = i;
            //RCLCPP_INFO(this->get_logger(), "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
            //RCLCPP_INFO(this->get_logger(), "Received path point: (%d)", min_index);
            // 配列に保存
            min_distance_list.push_back(min_distance);
            min_distance_idx_list.push_back(i);
        }
        
    }
    // add find the index of path nearest to the current index from list 'min_distance_idx_lst'
    int tmp_idx_dis;
    int min_idx_distance = std::numeric_limits<int>::max();
    for (size_t j=0; j < min_distance_idx_list.size() ;j++){
        tmp_idx_dis = min_distance_idx_list[j]-oldNearestPointIndex;
        if ( min_idx_distance > tmp_idx_dis){
            if (tmp_idx_dis < 100 ){
                min_index = min_distance_idx_list[j];
                //RCLCPP_INFO(this->get_logger(), "CCCC path point: (%ld)", min_distance_idx_list.size());
    
            }
        }
    }
    //RCLCPP_INFO(this->get_logger(), "index of path point: (%ld)", min_distance_idx_list.size());
    return min_index;
}

std::pair<int, double> PurePursuitComponent::searchTargetIndex() {
    if (odom_sub_flag){
        double Lf = calcLf(cfg_.k, current_velocity_, cfg_.Lfc);
        //RCLCPP_INFO(this->get_logger(), "Lf: %lf", Lf);
        if (oldNearestPointIndex == -1) {
            oldNearestPointIndex = calcFirstNearestPointIndex();
        } else {
            oldNearestPointIndex = calcOldNearestPointIndex();
        }

        int ind = oldNearestPointIndex;

        while (Lf > calcDistance(current_pose_.x, current_pose_.y, cx_[ind], cy_[ind])) {
            if (ind + 1 >= static_cast<int>(cx_.size())) {
                break;
            }
            ind++;
        }
        return { ind, Lf };
    }else{
        std::cout << "[WARN] searchTargetIndex() called before path was set." << std::endl;
    }
}

double PurePursuitComponent::calcDistance(double x1, double y1, double x2, double y2) const {
    return std::hypot(x2 - x1, y2 - y1);
}

} // namespace pure_pursuit_planner
