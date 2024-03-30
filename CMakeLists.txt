cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# other pakage
#find_package(path_smoother REQUIRED)

# pure_pursuit_planner
add_executable(pure_pursuit_planner src/pure_pursuit_planner_node.cpp src/pure_pursuit_planner_component.cpp)
ament_target_dependencies(pure_pursuit_planner
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2_geometry_msgs 
  tf2_ros
  #path_smoother
)
target_include_directories(pure_pursuit_planner PUBLIC
  include
  #${path_smoother_INCLUDE_DIRS}
)

# Install Cpp executables
install(TARGETS
  pure_pursuit_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

#include_directories(${path_smoother_INCLUDE_DIRS})

# include ディレクトリをエクスポート
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
