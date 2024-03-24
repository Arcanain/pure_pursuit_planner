## 機能概要 (Functional Overview)
Pure Pursuitアルゴリズムを使用して、ロボットまたは自動運転車のパス追従制御を行うためのROS 2ノードである。指定された経路に沿ってロボットを効率的に追従するための速度と角速度の指令を計算し出力する。

This software implements a ROS 2 node for path following control of robots or autonomous vehicles using the Pure Pursuit algorithm. It calculates and outputs velocity and angular velocity commands to efficiently navigate the robot along a specified path.

## IF表 (Interface Table)

### Input

| 変数名 (Variable Name)      | 型 (Type)            | 説明 (Description)                         |
|-------------------------|-------------------|---------------------------------------|
| `odom`                  | `nav_msgs::msg::Odometry` | ロボットのオドメトリ情報 (Odometry information of the robot) |

### Output

| 変数名 (Variable Name)      | 型 (Type)            | 説明 (Description)                         |
|-------------------------|-------------------|---------------------------------------|
| `cmd_vel`               | `geometry_msgs::msg::Twist` | ロボットの速度と角速度の指令 (Velocity and angular velocity commands for the robot) |
| `target_path`           | `nav_msgs::msg::Path` | 追従する経路 (Path to follow) |

### Internal Values

| 変数名 (Variable Name)      | 型 (Type)            | 説明 (Description)                         |
|-------------------------|-------------------|---------------------------------------|
| `x`, `y`, `yaw`         | `double`          | ロボットの現在位置と向き (Current position and orientation of the robot) |
| `v`, `w`                | `double`          | ロボットの速度と角速度 (Velocity and angular velocity of the robot) |
| `cx`, `cy`              | `std::vector<double>` | 経路のx座標とy座標のリスト (List of x and y coordinates of the path) |
| `target_ind`            | `int`             | 現在のターゲットインデックス (Current target index) |
| `target_vel`            | `double`          | 目標速度 (Target velocity) |
| `goal_threshold`        | `double`          | ゴール判定のしきい値 (Threshold for goal judgment) |
| `k`, `Lfc`, `Kp`, `dt`  | `double`          | Pure Pursuitパラメータ (Pure Pursuit parameters) |
| `oldNearestPointIndex`  | `int`             | 前回の最近点インデックス (Index of the nearest point in the previous iteration) |
| `current_vel`           | `double`          | ロボットの現在の速度 (Current velocity of the robot) |

## システム構成図 (System Configuration Diagram)

## 機能要件 (Functional Requirements)

| 機能 (Feature)                    | 要件 (Requirement)                                     | 理由 (Reason)                                             | 仕様 (Specification)                                                                                                                                                                 | 実関数 (Implemented Functions)                          |
|---------------------------------|------------------------------------------------------|---------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------|
| **パス追従制御 (Path Following Control)** | ロボットは指定された経路に沿って効率的に移動すること。 (The robot needs to efficiently move along the specified path.) | ロボットが目標に向かって正確に進むため。 (To ensure the robot accurately progresses towards the target.) | - Pure Pursuitアルゴリズムを使用して、ロボットの現在位置から見たターゲットポイントへの角度を計算し、それに基づいて速度と角速度を決定する。<br>(Implemented in `purePursuitControl`)<br>- 経路上でロボットから一定距離 (`Lf`) 先にあるターゲットポイントのインデックスを探索する。距離は速度に応じて動的に変化する。<br>(Implemented in `searchTargetIndex`) | `updateControl`, `purePursuitControl`, `searchTargetIndex` |
| **速度指令出力 (Velocity Command Output)** | ロボットは計算された速度と角速度の指令を出力すること。 (The robot needs to output the computed velocity and angular velocity commands.) | ロボットが適切な速度で進むため。 (To ensure the robot progresses at the appropriate speed.) | - 計算された速度 (`v`) と角速度 (`w`) を`geometry_msgs::msg::Twist`メッセージとして出力する。<br>(Implemented in `publishCmd`)<br>- ゴールに近づいた場合 (`goal_threshold` 未満) は速度と角速度をゼロにして停止する。<br>(Implemented in `publishCmd`) | `publishCmd`                                   |
| **経路出力 (Path Output)**              | ロボットは追従する経路を出力すること。 (The robot needs to output the path to follow.)               | ナビゲーションのための参照経路を提供するため。 (To provide a reference path for navigation.) | - 経路の各ポイント (`cx[i]`, `cy[i]`) を`nav_msgs::msg::Path`メッセージの一部として出力する。各ポイントは`geometry_msgs::msg::PoseStamped`メッセージとして格納される。<br>(Implemented in `publishPath`) | `publishPath`                                  |
| **オドメトリ受信 (Odometry Reception)**  | ロボットはオドメトリ情報を受信すること。 (The robot needs to receive odometry information.)         | ロボットの現在位置と向きを更新するため。 (To update the current position and orientation of the robot.) | - 受信した`nav_msgs::msg::Odometry`メッセージからロボットの現在位置 (`x`, `y`) と向き (`yaw`) を抽出し、内部変数を更新する。<br>(Implemented in `odometry_callback`) | `odometry_callback`                           |

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
