cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_planner)

enable_testing() 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# pure_pursuit_planner
add_library(pure_pursuit_planner_library
  src/pure_pursuit_planner_node.cpp
  src/pure_pursuit_planner_component.cpp
)
add_executable(pure_pursuit_planner src/pure_pursuit_planner.cpp)
ament_target_dependencies(pure_pursuit_planner_library
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2_geometry_msgs 
  tf2_ros
  visualization_msgs
)
target_include_directories(pure_pursuit_planner_library PUBLIC
  include
)


target_link_libraries(pure_pursuit_planner PRIVATE pure_pursuit_planner_library)

# Install Cpp executables
install(TARGETS
  pure_pursuit_planner
  pure_pursuit_planner_library
  DESTINATION lib/${PROJECT_NAME}
)

# YAML 設定ファイルをインストール
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

#==========================================================
# テストターゲットの定義
#==========================================================
if(BUILD_TESTING)
  # ament_add_gtest(test_dwa_component
  #   test/test_dwa_component.cpp
  # )
  # if(TARGET test_dwa_component)
  #   target_link_libraries(test_dwa_component pure_pursuit_planner_library)
  #   target_compile_options(test_dwa_component PUBLIC -fprofile-arcs -ftest-coverage)
  #   target_link_options(test_dwa_component PUBLIC -fprofile-arcs -ftest-coverage)

  #   target_include_directories(test_dwa_component PUBLIC
  #     ${CMAKE_CURRENT_SOURCE_DIR}/include
  #   )
  #   ament_target_dependencies(test_dwa_component
  #     rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
  #   )
  # endif()

  # ── CSV ベースの全関数対応 CSV テストを追加 ───────────────────
  ament_add_gtest(test_pp_all_functions
    test/test_all_functions_csv.cpp
  )
  if (TARGET test_pp_all_functions)
    target_link_libraries(test_pp_all_functions pure_pursuit_planner_library)
    target_compile_features(test_pp_all_functions PUBLIC cxx_std_14)
    target_include_directories(test_pp_all_functions PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )

    ament_target_dependencies(test_pp_all_functions
      rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
    )

    set_tests_properties(test_pp_all_functions PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/test_data"
    )
  endif()

  # カバレッジ生成ターゲットの追加
  find_program(LCOV_EXEC lcov)
  find_program(GENHTML_EXEC genhtml)
  if(LCOV_EXEC AND GENHTML_EXEC)
    add_custom_target(coverage
      COMMAND ${LCOV_EXEC} --directory . --capture --output-file coverage.info
      COMMAND ${LCOV_EXEC} --remove coverage.info '/usr/*' '/opt/ros/*' --output-file coverage.filtered.info
      COMMAND ${GENHTML_EXEC} coverage.filtered.info --output-directory coverage_html
      COMMENT "Generating coverage report..."
    )
  endif()
endif()

set(CMAKE_BUILD_TYPE Debug)

#==========================================================
# Lintやカバレッジ設定 (オプション)
#==========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
