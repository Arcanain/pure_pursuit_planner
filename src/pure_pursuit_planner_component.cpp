#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

PurePursuitNode::PurePursuitNode()
: Node("pure_pursuit_planner") {
    // Parameter setting
    // cx = std::vector<double>(100);
    // cy = std::vector<double>(100);
    target_ind = 0;
    oldNearestPointIndex = -1;
    //target_vel = 10.0 / 3.6;
    target_vel = 0.25;

    // Publisher
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("target_path", 10);
    path_pub_debug = this->create_publisher<nav_msgs::msg::Path>("new_path", 10);

    // Subscriber
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&PurePursuitNode::odometry_callback, this, _1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "tgt_path", 10,
            std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
    
    // Timer callback
    timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt * 1000)),
                                    std::bind(&PurePursuitNode::updateControl, this));
    // std::iota(cx.begin(), cx.end(), 0.0);
    // std::transform(cx.begin(), cx.end(), cy.begin(), [](double ix) { return std::sin(ix / 5.0) * ix / 2.0; });
    // std::tie(target_ind, std::ignore) = searchTargetIndex();

    // 2024/3/30
    // Load path data from CSV file
    // std::string file_path = "/ros2_ws/src/path_smoother/path/simulation_path.csv";
    // loadPathData(file_path);

    // // Create path message
    // path_.header.stamp = this->now();
    // path_.header.frame_id = "map";
    // path_.poses = poses_;

    // // Publish path at a fixed rate
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(20),  // 50 Hz
    //     std::bind(&PurePursuitNode::publishPathdebug, this));
}

// void PurePursuitNode::loadPathData(const std::string &file_path)
// {
//     std::ifstream file(file_path);
//     std::string line;
//     std::getline(file, line);  // Skip the header

//     while (std::getline(file, line))
//     {
//         std::stringstream ss(line);
//         std::string value;
//         std::vector<double> pose_data;

//         while (std::getline(ss, value, ','))
//         {
//             pose_data.push_back(std::stod(value));
//         }

//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.stamp = this->now();
//         pose.header.frame_id = "map";
//         pose.pose.position.x = pose_data[0];
//         pose.pose.position.y = pose_data[1];
//         pose.pose.position.z = pose_data[2];
//         pose.pose.orientation.x = pose_data[3];
//         pose.pose.orientation.y = pose_data[4];
//         pose.pose.orientation.z = pose_data[5];
//         pose.pose.orientation.w = pose_data[6];
//         poses_.push_back(pose);

//         // Add x and y coordinates to cx and cy vectors
//         cx.push_back(pose_data[0]);
//         cy.push_back(pose_data[1]);
//     }
// }

// void PurePursuitNode::publishPathdebug()
// {
//     path_pub->publish(path_);
//     // std_msgs::msg::Int32 path_num_msg;
//     // path_num_msg.data = static_cast<int>(poses_.size());
//     // path_num_pub_->publish(path_num_msg);
// }

void PurePursuitNode::updateControl() {
    // if (T < 0 || target_ind >= static_cast<int>(cx.size()) - 1) {
    //     rclcpp::shutdown();
    //     return;
    // }

    auto [v, w] = purePursuitControl(target_ind);

    publishCmd(v, w);
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
    double v = target_vel;
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

// void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
//     // cx.clear();
//     // cy.clear();
//     std::cout << msg->pose.pose.position.x << std::endl;
//     for (const auto& pose : msg->poses) {
//         cx.push_back(pose.pose.position.x);
//         cy.push_back(pose.pose.position.y);
//         std::cout << pose.pose.position.x << std::endl;
//     }
//     // Reset the target index and nearest point index
//     // target_ind = 0;
//     // oldNearestPointIndex = -1;
// }

void PurePursuitNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // 受け取ったパスをクリア
    // cx.clear();
    // cy.clear();

    // 受け取ったパスのサイズに合わせてcxとcyのサイズを調整
    // cx.resize(msg->poses.size());
    // cy.resize(msg->poses.size());

    // std::iota(cx.begin(), cx.end(), 0.0);

    // 受け取ったパスメッセージから座標を抽出
    // for (size_t i = 0; i < msg->poses.size(); ++i) {
    //     cx[i] = msg->poses[i].pose.position.x;
    //     cy[i] = msg->poses[i].pose.position.y;
    // }

    // // 最新のパスに基づいて目標インデックスを更新
    // std::tie(target_ind, std::ignore) = searchTargetIndex();

    // std::iota(cx.begin(), cx.end(), 0.0);

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

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";

    for (const auto& pose : msg->poses) {
        geometry_msgs::msg::PoseStamped tmpPose;
        tmpPose.header.stamp = path_msg.header.stamp;
        tmpPose.header.frame_id = path_msg.header.frame_id;
        tmpPose.pose.position.x = pose.pose.position.x;
        tmpPose.pose.position.y = pose.pose.position.y;
        tmpPose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(tmpPose);

        // cx.push_back(pose.pose.position.x);
        // cy.push_back(pose.pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "Received path point: (%f, %f)", pose.pose.position.x, pose.pose.position.y);
    }

    path_pub_debug->publish(path_msg);

    // Create a new Path message
    // nav_msgs::msg::Path new_path_msg;
    // new_path_msg.header = msg->header;

    // // Modify each pose in the path
    // for (const auto& pose : msg->poses)
    // {
    //     geometry_msgs::msg::PoseStamped modified_pose = pose;
    //     modified_pose.pose.position.z = 1.0; // Set the z coordinate to 1.0
    //     new_path_msg.poses.push_back(modified_pose);
    // }

    // // Publish the modified path to the 'new_path' topic
    // path_pub_debug->publish(new_path_msg);
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
