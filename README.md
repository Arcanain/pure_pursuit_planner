## 機能概要 (Functional Overview)
Pure Pursuitアルゴリズムを使用して、ロボットまたは自動運転車のパス追従制御を行うためのROS 2ノードである。  
指定された経路に沿ってロボットを効率的に追従するための速度と角速度の指令を計算し出力する。

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

以下に、要件に対する理由を追加し、要件の文言を修正した上で、仕様と関数の関連を明確にした機能要件表を示します。

---

## 機能要件 (Functional Requirements)

| 機能 (Feature)                    | 要件 (Requirement)                                     | 理由 (Reason)                                             | 仕様 (Specification)                                                                                                                                                                 | 実関数 (Implemented Functions)                          |
|---------------------------------|------------------------------------------------------|---------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------|
| **パス追従制御 (Path Following Control)** | ロボットは指定された経路に沿って効率的に移動すること。 (The robot needs to efficiently move along the specified path.) | ロボットが目標に向かって正確に進むため。 (To ensure the robot accurately progresses towards the target.) | - Pure Pursuitアルゴリズムを使用して、ターゲットポイントを追従する。<br>(Implemented in `purePursuitControl`)<br>- 経路上のターゲットポイントのインデックスを探索する。<br>(Implemented in `searchTargetIndex`) | `updateControl`, `purePursuitControl`, `searchTargetIndex` |
| **速度指令出力 (Velocity Command Output)** | ロボットは計算された速度と角速度の指令を出力すること。 (The robot needs to output the computed velocity and angular velocity commands.) | ロボットが適切な速度で進むため。 (To ensure the robot progresses at the appropriate speed.) | - 計算された速度と角速度を`geometry_msgs::msg::Twist`メッセージとして出力する。<br>(Implemented in `publishCmd`)<br>- ゴールに近づいた場合は停止する。<br>(Implemented in `publishCmd`) | `publishCmd`                                   |
| **経路出力 (Path Output)**              | ロボットは追従する経路を出力すること。 (The robot needs to output the path to follow.)               | ナビゲーションのための参照経路を提供するため。 (To provide a reference path for navigation.) | - 経路を`nav_msgs::msg::Path`メッセージとして出力する。<br>(Implemented in `publishPath`)                                                                                               | `publishPath`                                  |
| **オドメトリ受信 (Odometry Reception)**  | ロボットはオドメトリ情報を受信すること。 (The robot needs to receive odometry information.)         | ロボットの現在位置と向きを更新するため。 (To update the current position and orientation of the robot.) | - オドメトリ情報を`nav_msgs::msg::Odometry`メッセージとして受信する。<br>(Implemented in `odometry_callback`)<br>- 受信したオドメトリ情報からロボットの現在位置と向きを更新する。<br>(Implemented in `odometry_callback`) | `odometry_callback`                           |