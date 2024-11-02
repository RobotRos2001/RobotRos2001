cmake_minimum_required(VERSION 3.8)
project(custom_joint_position_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)  # 添加 control_msgs 依赖

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(joint_pub_position src/joint_pub_position.cpp)
ament_target_dependencies(joint_pub_position rclcpp std_msgs sensor_msgs control_msgs)  # 包含 control_msgs

install(TARGETS joint_pub_position DESTINATION lib/${PROJECT_NAME})

ament_package()
