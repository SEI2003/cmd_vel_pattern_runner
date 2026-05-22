cmake_minimum_required(VERSION 3.8)
project(kaitenyaki_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 1. 実行ファイル（バイナリ）を作る指示
# odm という実行ファイルを src/odm.cpp からビルドする
add_executable(odm src/odm.cpp)

# 2. ROS2(rclcpp)のライブラリをリンクする指示
ament_target_dependencies(odm rclcpp sensor_msgs nav_msgs)

# 3. ros2 run コマンドで見つけられる場所にインストールする指示
install(TARGETS
  odm
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(odm_py src/odm_py.cpp)

ament_target_dependencies(odm_py rclcpp sensor_msgs nav_msgs)

install(TARGETS
  odm_py
  DESTINATION lib/${PROJECT_NAME}
)

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
