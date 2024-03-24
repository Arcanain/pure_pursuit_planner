## Functional Overview
This software implements a ROS 2 node for path following control of robots or autonomous vehicles using the Pure Pursuit algorithm. It calculates and outputs velocity and angular velocity commands to efficiently navigate the robot along a specified path.

## Interface Table

### Input

| Variable Name      | Type            | Description                         |
|-------------------------|-------------------|---------------------------------------|
| `odom`                  | `nav_msgs::msg::Odometry` | Odometry information of the robot |

### Output

| Variable Name      | Type            | Description                         |
|-------------------------|-------------------|---------------------------------------|
| `cmd_vel`               | `geometry_msgs::msg::Twist` | Velocity and angular velocity commands for the robot |
| `target_path`           | `nav_msgs::msg::Path` | Path to follow |

### Internal Values

| Variable Name      | Type            | Description                         |
|-------------------------|-------------------|---------------------------------------|
| `x`, `y`, `yaw`         | `double`          | Current position and orientation of the robot |
| `v`, `w`                | `double`          | Velocity and angular velocity of the robot |
| `cx`, `cy`              | `std::vector<double>` | List of x and y coordinates of the path |
| `target_ind`            | `int`             | Current target index |
| `target_vel`            | `double`          | Target velocity |
| `goal_threshold`        | `double`          | Threshold for goal judgment |
| `k`, `Lfc`, `Kp`, `dt`  | `double`          | Pure Pursuit parameters |
| `oldNearestPointIndex`  | `int`             | Index of the nearest point in the previous iteration |
| `current_vel`           | `double`          | Current velocity of the robot |

## System Configuration Diagram

```mermaid
flowchart TD
    A[Start] --> B[Initialize ROS 2 Node: pure_pursuit_planner]
    B --> C[Create Publishers and Subscribers]
    C --> D[Enter Timer Callback Loop]
    D --> E[Update Control]
    E --> F[Search Target Index]
    F --> G[Pure Pursuit Control]
    G --> H[Publish Command Velocity]
    H --> I[Publish Target Path]
    I --> J[Odometry Callback]
    J --> D

    subgraph PurePursuitNode
        K[Constructor: Initialize Node and Topics]
        L[updateControl: Main Logic for Velocity Command]
        M[purePursuitControl: Calculate Steering and Velocity]
        N[searchTargetIndex: Find Next Waypoint]
        O[publishCmd: Publishes Velocity Command]
        P[publishPath: Publishes Target Path for Visualization]
        Q[odometry_callback: Updates Current Pose]
        R[convertScanToPointCloud and performKMeansClustering: Not Implemented in This Snippet]
    end

    K --> L
    L --> M
    M --> N
    N --> O
    O --> P
    P --> Q
```

## Functional Requirements

| Feature                    | Requirement                                            | Reason                                                      | Specification                                                                                                                                                                | Implemented Functions                          |
|----------------------------|--------------------------------------------------------|-------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------|
| **Path Following Control** | The robot needs to efficiently move along the specified path. | To ensure the robot accurately progresses towards the target. | - Use the Pure Pursuit algorithm to calculate the angle to the target point from the robot's current position and determine the velocity and angular velocity based on that.<br>(Implemented in `purePursuitControl`)<br>- Search for the index of the target point on the path that is a certain distance (`Lf`) ahead of the robot. The distance dynamically changes according to the velocity.<br>(Implemented in `searchTargetIndex`) | `updateControl`, `purePursuitControl`, `searchTargetIndex` |
| **Velocity Command Output** | The robot needs to output the computed velocity and angular velocity commands. | To ensure the robot progresses at the appropriate speed. | - Output the computed velocity (`v`) and angular velocity (`w`) as a `geometry_msgs::msg::Twist` message.<br>(Implemented in `publishCmd`)<br>- Stop by setting the velocity and angular velocity to zero if approaching the goal (`goal_threshold` or less).<br>(Implemented in `publishCmd`) | `publishCmd`                                   |
| **Path Output**             | The robot needs to output the path to follow.             | To provide a reference path for navigation.                 | - Output each point of the path (`cx[i]`, `cy[i]`) as part of a `nav_msgs::msg::Path` message. Each point is stored as a `geometry_msgs::msg::PoseStamped` message.<br>(Implemented in `publishPath`) | `publishPath`                                  |
| **Odometry Reception**      | The robot needs to receive odometry information.          | To update the current position and orientation of the robot. | - Extract the robot's current position (`x`, `y`) and orientation (`yaw`) from the received `nav_msgs::msg::Odometry` message and update the internal variables.<br>(Implemented in `odometry_callback`) | `odometry_callback`                           |

## 詳細設計 (Detailed Design)

| メソッド名 (Method Name) | 目的 (Purpose)                           | 処理内容 (Process)                                                                                                          |
|-----------------------|----------------------------------------|---------------------------------------------------------------------------------------------------------------------------|
| `updateControl`       | 制御ループを実行し、速度指令と追従経路を更新する。    | 1. `purePursuitControl`メソッドを呼び出して速度と角速度を計算する。<br>2. `publishCmd`メソッドを呼び出して速度指令をパブリッシュする。<br>3. `publishPath`メソッドを呼び出して追従経路をパブリッシュする。 |
| `purePursuitControl`  | Pure Pursuitアルゴリズムに基づいて速度と角速度を計算する。 | 1. `searchTargetIndex`メソッドを呼び出してターゲットポイントのインデックスを探索する。<br>2. ターゲットポイントに対する角度を計算する。<br>3. 計算された角度とパラメータに基づいて速度と角速度を計算する。 |
| `publishPath`         | 追従経路をパブリッシュする。                   | 1. `nav_msgs::msg::Path`メッセージを作成する。<br>2. 経路の各ポイントを`Path`メッセージに追加する。<br>3. `path_pub`を使用して`Path`メッセージをパブリッシュする。 |
| `searchTargetIndex`   | ターゲットポイントのインデックスを探索する。             | 1. ロボットから一定距離 (`Lf`) 先にあるターゲットポイントのインデックスを探索する。<br>2. 距離は速度に応じて動的に変化する。 |
| `calcDistance`        | 点とロボットの現在位置との距離を計算する。             | 1. ロボットの現在位置 (`x`, `y`) と指定された点 (`point_x`, `point_y`) とのユークリッド距離を計算する。 |
| `odometry_callback`   | オドメトリ情報のコールバック関数。                | 1. 受信した`nav_msgs::msg::Odometry`メッセージからロボットの現在位置 (`x`, `y`) と向き (`yaw`) を抽出し、内部変数を更新する。 |
| `publishCmd`          | 速度指令をパブリッシュする。                     | 1. `geometry_msgs::msg::Twist`メッセージを作成し、計算された速度 (`v`) と角速度 (`w`) を設定する。<br>2. `cmd_vel_pub`を使用して`Twist`メッセージをパブリッシュする。<br>3. ゴールに近づいた場合 (`goal_threshold` 未満) は速度と角速度をゼロにして停止する。 |
