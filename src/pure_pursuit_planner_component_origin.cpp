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
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    look_ahead_range_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "look_ahead_range_marker",
      10);
    lidar_range_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "lidar_range_marker",
      10);

    obstacle_range_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "obstacle_range_marker",
      10);

    target_point_pub = this->create_publisher<visualization_msgs::msg::Marker>("target_point_marker", 10);

    
    // Subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "tgt_path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));

    local_obstacle_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "local_obstacle_markers", 10,
            std::bind(&PurePursuitNode::local_obstacle_callback, this, _1));
    
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

    if (!obstacle_detected){
        double target_lookahed_x, target_lookahed_y, target_curvature;
        if (ind < static_cast<int>(cx.size())) {
            target_lookahed_x =cx[ind];
            target_lookahed_y = cy[ind];
            target_curvature = ck[ind];
        } else {
            target_lookahed_x = cx.back();
            target_lookahed_y = cy.back();
            target_curvature = ck.back();
            ind = static_cast<int>(cx.size()) - 1;
        }
    }
    // target speed
    double curvature = std::max(minCurvature, std::min(abs(target_curvature), maxCurvature));
    curvature = curvature / maxCurvature;
    double target_vel = (maxVelocity - minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + minVelocity; //[m/s]

    double alpha = std::atan2(target_lookahed_y - y, target_lookahed_x - x) - yaw;
    double v = target_vel;
    double w = v * std::tan(alpha) / Lf;
    

    //注視点の可視化
    visualizeTargetPoint(target_lookahed_x, target_lookahed_y);
    visualizeTargetCircle(target_lookahed_x, target_lookahed_y);
    

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

    // マーカーの色を設定 (青色)
    target_point_marker.color.r = 0.0;
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
    if (obstacle_detected){
        for (const auto& marker : msg->markers) {
            
            // 障害物の中心座標を取得
            double obstacle_x = marker.pose.position.x;
            double obstacle_y = marker.pose.position.y;

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
            double radius = 0.5;  // 円の半径
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

        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
    } else {
        cmd_vel_msg.linear.x = v;
        cmd_vel_msg.angular.z = w;
    }

    cmd_vel_pub->publish(cmd_vel_msg);
}