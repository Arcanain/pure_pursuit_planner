## Functional Overview
This software implements a ROS 2 node for path following control of robots or autonomous vehicles using the Pure Pursuit algorithm. It calculates and outputs velocity and angular velocity commands to efficiently navigate the robot along a specified path.

![Peek 2024-03-30 23-45](https://github.com/Arcanain/pure_pursuit_planner/assets/52307432/19483a1f-92bd-49bc-9e26-91188e22c41b)

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

## Detailed Design

| Method Name           | Purpose                                           | Process                                                                                                                                    |
|-----------------------|---------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| `updateControl`       | To execute the control loop and update the velocity command and following path. | 1. Call the `purePursuitControl` method to calculate the velocity and angular velocity.<br>2. Call the `publishCmd` method to publish the velocity command.<br>3. Call the `publishPath` method to publish the following path. |
| `purePursuitControl`  | To calculate the velocity and angular velocity based on the Pure Pursuit algorithm. | 1. Call the `searchTargetIndex` method to search for the index of the target point.<br>2. Calculate the angle to the target point.<br>3. Calculate the velocity and angular velocity based on the calculated angle and parameters. |
| `publishPath`         | To publish the following path.                        | 1. Create a `nav_msgs::msg::Path` message.<br>2. Add each point of the path to the `Path` message.<br>3. Use `path_pub` to publish the `Path` message. |
| `searchTargetIndex`   | To search for the index of the target point.         | 1. Search for the index of the target point that is a certain distance (`Lf`) ahead of the robot.<br>2. The distance dynamically changes according to the velocity. |
| `calcDistance`        | To calculate the distance between a point and the robot's current position. | 1. Calculate the Euclidean distance between the robot's current position (`x`, `y`) and the specified point (`point_x`, `point_y`). |
| `odometry_callback`   | Callback function for odometry information.          | 1. Extract the robot's current position (`x`, `y`) and orientation (`yaw`) from the received `nav_msgs::msg::Odometry` message and update the internal variables. |
| `publishCmd`          | To publish the velocity command.                     | 1. Create a `geometry_msgs::msg::Twist` message and set the calculated velocity (`v`) and angular velocity (`w`).<br>2. Use `cmd_vel_pub` to publish the `Twist` message.<br>3. Stop by setting the velocity and angular velocity to zero if approaching the goal (`goal_threshold` or less). |
