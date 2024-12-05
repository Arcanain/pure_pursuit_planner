#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode()
: Node("pure_pursuit_planner") {
    // Parameter setting
    target_ind = 0;
    oldNearestPointIndex = -1;

    // Publisher
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_tmp", 10);
    look_ahead_range_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "look_ahead_range_marker",
      10);
    lidar_range_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "lidar_range_marker",
      10);

    obstacle_range_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "obstacle_range_marker",
      10);

    goal_status_pub = this->create_publisher<std_msgs::msg::Bool>("goal_status", 10);

    target_point_pub = this->create_publisher<visualization_msgs::msg::Marker>("target_point_marker", 10);

    
    // Subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "tgt_path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));

    //simulation
    local_obstacle_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "local_obstacle_markers", 10,
            std::bind(&PurePursuitNode::local_obstacle_callback, this, _1));

    //Lidar
    /*local_obstacle_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "closest_point_marker", 10,
            std::bind(&PurePursuitNode::local_obstacle_callback, this, _1));*/
            
    obstacle_detected_sub = this->create_subscription<std_msgs::msg::Bool>(
            "obstacle_detected", 10,
            std::bind(&PurePursuitNode::obstacle_detected_callback, this, _1));

    current_time = this->get_clock()->now();
    // Timer callback
    timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                    std::bind(&PurePursuitNode::updateControl, this));
}

void PurePursuitNode::updateControl() {
    current_time = this->get_clock()->now();
    if (path_subscribe_flag && odom_subscribe_flag) {
        auto [v, w] = purePursuitControl(target_ind);
        publishCmd(v, w);

        // 円の線を描くマーカーを作成
        visualization_msgs::msg::Marker line_strip_marker;
        line_strip_marker.header.frame_id = "base_link";
        line_strip_marker.header.stamp = current_time;
        line_strip_marker.ns = "circle_line";
        line_strip_marker.id = 1;
        line_strip_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip_marker.action = visualization_msgs::msg::Marker::ADD;
        line_strip_marker.pose.orientation.w = 1.0;
        line_strip_marker.scale.x = 0.05;  // 線の太さ
        line_strip_marker.color.r = 0.0;
        line_strip_marker.color.g = 0.0;
        line_strip_marker.color.b = 1.0;
        line_strip_marker.color.a = 1.0;  // 不透明

        // 円周上の点を追加
        double radius = 1.0;
        int num_points = 100;
        for (int i = 0; i <= num_points; ++i) {
            double angle = i * 2.0 * M_PI / num_points;
            geometry_msgs::msg::Point p;
            p.x = radius * cos(angle);
            p.y = radius * sin(angle);
            p.z = 0.0;
            line_strip_marker.points.push_back(p);
        }

        // マーカーをパブリッシュ
        lidar_range_pub->publish(line_strip_marker);
    }
}

std::pair<double, double> PurePursuitNode::purePursuitControl(int& pind) {
    auto [ind, Lf] = searchTargetIndex();
    //ind:path配列の要素数
    if (pind >= ind) {
        ind = pind;
    }

    double target_lookahed_x, target_lookahed_y, target_curvature;
    if (ind < static_cast<int>(cx.size())) {
        RCLCPP_INFO(this->get_logger(), "注視点更新");
        target_lookahed_x =cx[ind];
        target_lookahed_y = cy[ind];
        target_curvature = ck[ind];
        RCLCPP_INFO(this->get_logger(), "ind:%d",ind);
        RCLCPP_INFO(this->get_logger(), "x,y:%lf,%lf",target_lookahed_x, target_lookahed_y);
    } else {
        RCLCPP_INFO(this->get_logger(), "注視点更新なし");
        target_lookahed_x = cx.back();
        target_lookahed_y = cy.back();
        target_curvature = ck.back();
        ind = static_cast<int>(cx.size()) - 1;
    }
    
    if (obstacle_detected || avoidance_flag){
        //ロボットと障害物との距離

        //Lidarの場合
        /*obstacle_x += x;
        obstacle_y += y;*/

        double lenRobotObstacle = calcDistance(obstacle_x, obstacle_y);
        //double lenRobotObstacle = calcDistance(obstacle_x, obstacle_y);
        RCLCPP_INFO(this->get_logger(), "回避開始");
        RCLCPP_INFO(this->get_logger(), "%lf,%lf",obstacle_x, obstacle_y);
        RCLCPP_INFO(this->get_logger(), "%lf",lenRobotObstacle);
        if (lenRobotObstacle < Lf + obstacle_th){
            RCLCPP_INFO(this->get_logger(), "回避開始2");
            avoidance_flag = true;
            //回避行動処理
            if (temp_target_x != 0 && temp_target_y != 0){
                target_curvature = 1.5;
                auto [min_distance, closest_point] = calcClosestPointOnPath();
                double closest_x = closest_point.first;
                double closest_y = closest_point.second;
                diff_min_dist = std::hypot(closest_x - init_x, closest_y - init_y);
                if (min_distance < 0.3 && diff_min_dist > 0.5){
                    temp_target_x = 0;
                    temp_target_y = 0;
                    pre_min_distance = 0;
                    avoidance_flag = false;
                    RCLCPP_INFO(this->get_logger(), "回避終了");
                }else{
                    target_lookahed_x = temp_target_x;
                    target_lookahed_y = temp_target_y;
                }

                pre_min_distance = min_distance;

            }else{
                init_x = target_lookahed_x;
                init_y = target_lookahed_y;
            }

            if (avoidance_flag){
                RCLCPP_INFO(this->get_logger(), "回避開始3");
                double x0, y0, xr, yr, th;
                x0 = obstacle_x;
                y0 = obstacle_y;
                xr = x;
                yr = y;
                th = obstacle_th;

                double A = 1 + std::pow((yr - y0) / (x0 - xr), 2);
                double B = -2 * yr - (2 * xr * (yr - y0) / (x0 - xr))
                    + ((yr - y0) / (x0 - xr)) * ((std::pow(Lf, 2) + std::pow(x0, 2) + std::pow(y0, 2) - std::pow(th, 2) - std::pow(xr, 2) - std::pow(yr, 2)) / (x0 - xr));
                double C = std::pow(xr, 2) + std::pow(yr, 2) - std::pow(Lf, 2)
                    + std::pow((std::pow(Lf, 2) + std::pow(x0, 2) + std::pow(y0, 2) - std::pow(th, 2) - std::pow(xr, 2) - std::pow(yr, 2)) / (2 * x0 - 2 * xr), 2)
                    - xr * (std::pow(Lf, 2) + std::pow(x0, 2) + std::pow(y0, 2) - std::pow(th, 2) - std::pow(xr, 2) - std::pow(yr, 2)) / (x0 - xr);
                
                double ylp, xlp, Lp, xlm, ylm, Lm;
                //2つの交点を算出
                ylp = (-B + std::sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);
                xlp = ((yr - y0) / (x0 - xr)) * ylp + ((std::pow(Lf, 2) + std::pow(x0, 2) + std::pow(y0, 2) 
                    - std::pow(th, 2) - std::pow(xr, 2) - std::pow(yr, 2)) / (2 * x0 - 2 * xr));
                // 交点と前方注視点との距離
                Lp = std::sqrt(std::pow(target_lookahed_x - xlp, 2) + std::pow(target_lookahed_y - ylp, 2));

                ylm = (-B - std::sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);
                xlm = ((yr - y0) / (x0 - xr)) * ylm + ((std::pow(Lf, 2) + std::pow(x0, 2) + std::pow(y0, 2) 
                    - std::pow(th, 2) - std::pow(xr, 2) - std::pow(yr, 2)) / (2 * x0 - 2 * xr));
                // 交点と前方注視点との距離
                Lm = std::sqrt(std::pow(target_lookahed_x - xlm, 2) + std::pow(target_lookahed_y - ylm, 2));
                if (Lm < Lp){
                    target_lookahed_x = xlm;
                    target_lookahed_y = ylm;
                }else{
                    target_lookahed_x = xlp;
                    target_lookahed_y = ylp;
                }
                target_curvature = -2.5;
                temp_target_x = target_lookahed_x;
                temp_target_y = target_lookahed_y;

            }

        }
    }
    //注視点の可視化
    visualizeTargetPoint(target_lookahed_x, target_lookahed_y);
    visualizeTargetCircle(target_lookahed_x, target_lookahed_y);

    // target speed
    double curvature = std::max(minCurvature, std::min(abs(target_curvature), maxCurvature));
    curvature = curvature / maxCurvature;
    double target_vel = (maxVelocity - minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + minVelocity; //[m/s]

    // 速度と時間の差分を計算
    auto [acceleration_result, delta_time_result] = calcAcceleration(target_vel, current_time);
    double acceleration = acceleration_result;
    double delta_time = delta_time_result;
    //RCLCPP_INFO(this->get_logger(), "acceleration: %lf", acceleration);
    // 加速度制限の適用
    if(acceleration > max_acceleration){
        RCLCPP_INFO(this->get_logger(), "acceleration over: %lf", acceleration);
        double max_velocity_change = max_acceleration * delta_time;
        target_vel = previous_vel + max_velocity_change;
        auto [temp_acceleration_result, temp_delta_time] = calcAcceleration(target_vel, current_time);
        double limited_acceleration = temp_acceleration_result;
        RCLCPP_INFO(this->get_logger(), "limited_acceleration: %lf", limited_acceleration);
    }

    RCLCPP_INFO(this->get_logger(), "target_vel: %lf", target_vel*3.6);
    double alpha = std::atan2(target_lookahed_y - y, target_lookahed_x - x) - yaw;
    if (abs(alpha - M_PI) < 0.1){
        alpha = alpha + 0.15;
    }
    RCLCPP_INFO(this->get_logger(), "#### alpha: %lf", alpha);
    double v = target_vel;
    //double w = v * std::tan(alpha) / Lf;  
    double w = v* 2* std::sin(alpha) / Lf;
    
    /*
    if(abs(alpha) > (M_PI * 8 / 9)){
        RCLCPP_INFO(this->get_logger(), "limit alpha: %lf", w);
        w = 0.5;
    }*/
    if (std::isnan(w)) {
        RCLCPP_INFO(this->get_logger(), "limit nan: %lf", w);
        w = 0.5;
    }
    if (w > max_angular_velocity) {
        RCLCPP_INFO(this->get_logger(), "limit w: %lf", w);
        w = max_angular_velocity;
    } else if (w < -max_angular_velocity) {
        RCLCPP_INFO(this->get_logger(), "limit w: %lf", w);
        w = -max_angular_velocity;
    }
    //RCLCPP_INFO(this->get_logger(), "w: %lf", w);

    pind = ind;
    return { v, w };
}



std::pair<double, std::pair<double, double>> PurePursuitNode::calcClosestPointOnPath() {
    double min_distance = std::numeric_limits<double>::max();
    std::pair<double, double> closest_point;

    for (size_t i = 0; i < cx.size(); ++i) {
        double distance = calcDistance(cx[i], cy[i]);
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = {cx[i], cy[i]};
        }
    }

    return {min_distance, closest_point};
}

std::pair<double, double> PurePursuitNode::calcAcceleration(double current_vel, rclcpp::Time now_time) {
    // 速度の差分を計算
    double vel_diff = current_vel - previous_vel;

    // 時間の差分を計算 (秒単位)
    double time_diff = (now_time - previous_time).seconds();

    // 加速度 = 速度の変化量 / 時間の変化量
    double acceleration = vel_diff / time_diff;
    //RCLCPP_INFO(this->get_logger(), "acceleration: %lf", acceleration);
    if (acceleration < max_acceleration){
        // 前回の速度と時間を更新
        previous_vel = current_vel;
        previous_time = now_time;
    }

    return {acceleration, time_diff};
}


std::pair<int, double> PurePursuitNode::searchTargetIndex() {
    double Lf = k * v + Lfc;
    RCLCPP_INFO(this->get_logger(), "Lf: %lf", Lf);
    if (oldNearestPointIndex == -1) {
        double min_distance = std::numeric_limits<double>::max();
        int min_index = -1;
        for (size_t i = 0; i < cx.size(); i++) {
            double distance = calcDistance(cx[i], cy[i]);
            if (distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }
        oldNearestPointIndex = min_index;
    } else {
        bool count_flag = false;
        int count = 0, min_index = -1;
        double min_distance = std::numeric_limits<double>::max();
        
        std::vector<double> min_distance_list; 
        std::vector<int> min_distance_idx_list; 
        min_distance_list.clear();
        min_distance_idx_list.clear();

        
        
        for (size_t i = oldNearestPointIndex-20 ; i < cx.size(); i++)  {
            double distanceThisIndex = calcDistance(cx[i],     cy[i]);
            double distanceNextIndex = calcDistance(cx[i + 1], cy[i + 1]);
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
        RCLCPP_INFO(this->get_logger(), "index of path point: (%ld)", min_distance_idx_list.size());
        oldNearestPointIndex = min_index;
    }

    int ind = oldNearestPointIndex;
    while (Lf > calcDistance(cx[ind], cy[ind])) {
        if (ind + 1 >= static_cast<int>(cx.size())) {
            break;
        }
        ind++;
    }
    return { ind, Lf };
}



//前方注視点のドットを可視化
void PurePursuitNode::visualizeTargetPoint(double target_lookahed_x, double target_lookahed_y) {
    visualization_msgs::msg::Marker target_point_marker;
    
    // マーカーのヘッダーを設定
    target_point_marker.header.frame_id = "map";  // 適切なフレームに変更する
    target_point_marker.header.stamp = this->get_clock()->now();
    target_point_marker.ns = "target_point";
    target_point_marker.id = 0;  // 識別用ID
    target_point_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 点として表示するためにSPHEREを使用
    target_point_marker.action = visualization_msgs::msg::Marker::ADD;

    // マーカーの位置を設定
    target_point_marker.pose.position.x = target_lookahed_x;
    target_point_marker.pose.position.y = target_lookahed_y;
    target_point_marker.pose.position.z = 0.0;  // 2D平面上に表示するためZ座標は0

    // マーカーのサイズを設定
    double scale = 0.1;
    target_point_marker.scale.x = scale;  // 直径0.2mの点
    target_point_marker.scale.y = scale;
    target_point_marker.scale.z = scale;

    // マーカーの色を設定 (紫色)
    target_point_marker.color.r = 1.0;
    target_point_marker.color.g = 0.0;
    target_point_marker.color.b = 1.0;
    target_point_marker.color.a = 1.0;  // 不透明

    // マーカーをパブリッシュ
    target_point_pub->publish(target_point_marker);
}

void PurePursuitNode::visualizeTargetCircle(double target_lookahed_x, double target_lookahed_y) {
    visualization_msgs::msg::Marker circle_marker;
    
    // マーカーのヘッダーを設定
    circle_marker.header.frame_id = "base_link";  // ロボットを中心とするためのフレーム
    circle_marker.header.stamp = this->get_clock()->now();
    circle_marker.ns = "target_circle";
    circle_marker.id = 0;  // 識別用ID
    circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;  // 円を描くためにLINE_STRIPを使用
    circle_marker.action = visualization_msgs::msg::Marker::ADD;

    // 円の半径を計算 (ロボットとターゲットの距離)
    double radius = calcDistance(target_lookahed_x, target_lookahed_y);
    
    // 円周上の点を追加
    int num_points = 100; // 円を描くための点の数
    for (int i = 0; i <= num_points; ++i) {
        double angle = i * 2.0 * M_PI / num_points;
        geometry_msgs::msg::Point p;
        p.x = radius * cos(angle);
        p.y = radius * sin(angle);
        p.z = 0.0;
        circle_marker.points.push_back(p);
    }

    // マーカーのサイズを設定（線の太さ）
    circle_marker.scale.x = 0.05;  // 線の太さ

    // マーカーの色を設定 (青色)
    circle_marker.color.r = 0.0;
    circle_marker.color.g = 0.0;
    circle_marker.color.b = 1.0;
    circle_marker.color.a = 1.0;  // 不透明

    // マーカーをパブリッシュ
    look_ahead_range_pub->publish(circle_marker);
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

    odom_subscribe_flag = true;
}

void PurePursuitNode::obstacle_detected_callback(const std_msgs::msg::Bool::SharedPtr msg){
    obstacle_detected = msg->data;
}

void PurePursuitNode::local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "obstacle_detected_flag: %s", obstacle_detected ? "true" : "false");
    if (obstacle_detected || avoidance_flag){
        for (const auto& marker : msg->markers) {
            
            // 障害物の中心座標を取得
            obstacle_x = marker.pose.position.x;
            obstacle_y = marker.pose.position.y;

            // 円を描くマーカーを作成
            visualization_msgs::msg::Marker circle_marker;
            circle_marker.header.frame_id = marker.header.frame_id; // 修正点
            circle_marker.header.stamp = this->get_clock()->now();
            circle_marker.ns = "obstacle_circle";
            circle_marker.id = marker.id;  // 障害物IDと同じに設定
            circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            circle_marker.action = visualization_msgs::msg::Marker::ADD;
            circle_marker.pose.orientation.w = 1.0;
            circle_marker.scale.x = 0.05;  // 線の太さ
            circle_marker.color.r = 0.0;
            circle_marker.color.g = 1.0;
            circle_marker.color.b = 0.0;  
            circle_marker.color.a = 1.0;  // 不透明

            // 円周上の点を追加
            double radius = obstacle_th; // 円の半径
            int num_points = 100; // 円周上の点の数
            for (int i = 0; i <= num_points; ++i) {
                double angle = i * 2.0 * M_PI / num_points;
                geometry_msgs::msg::Point p;
                p.x = obstacle_x + radius * cos(angle);
                p.y = obstacle_y + radius * sin(angle);
                p.z = 0.0;
                circle_marker.points.push_back(p);
            }

            // 円を閉じるために最初の点を再度追加
            circle_marker.points.push_back(circle_marker.points.front());

            // マーカーをパブリッシュ
            obstacle_range_pub->publish(circle_marker);
        }
    }
    else{
            // Create and publish a DELETEALL marker individually
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

        obstacle_range_pub->publish(delete_marker);

    }
}


void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (!path_subscribe_flag) {
        // 以前のパス情報をクリア
        cx.clear();
        cy.clear();
        ck.clear();
        cyaw.clear();
        // 受け取ったパスメッセージから座標を抽出
        for (const auto& pose : msg->poses) {
            cx.push_back(pose.pose.position.x);
            cy.push_back(pose.pose.position.y);
            ck.push_back(pose.pose.position.z);

            tf2::Quaternion quat;
            tf2::fromMsg(pose.pose.orientation, quat);
            tf2::Matrix3x3 mat(quat);
            double roll_rev, pitch_rev, yaw_rev;
            mat.getRPY(roll_rev, pitch_rev, yaw_rev);
            cyaw.push_back(yaw_rev);

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
        goal_flag = true;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
    } else {
        if (goal_flag){
            v = 0.0;
            w = 0.0;
        }
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = w; 
    }

    std_msgs::msg::Bool goal_status_msg;
    goal_status_msg.data = goal_flag;
    goal_status_pub->publish(goal_status_msg);

    cmd_vel_pub->publish(cmd_vel_msg);
}