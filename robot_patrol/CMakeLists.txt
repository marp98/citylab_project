cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_patrol_msg REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES action_msgs geometry_msgs
)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
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

add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service rclcpp sensor_msgs geometry_msgs robot_patrol_msg)

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service rclcpp geometry_msgs sensor_msgs robot_patrol_msg)

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp sensor_msgs robot_patrol_msg)

add_executable(go_to_pose src/go_to_pose.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

if(cpp_typesupport_target)
  target_link_libraries(go_to_pose "${cpp_typesupport_target}")
endif()

ament_target_dependencies(go_to_pose rclcpp rclcpp_action geometry_msgs sensor_msgs nav_msgs action_msgs tf2)

install(TARGETS
   patrol_with_service
   direction_service
   test_service
   go_to_pose
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
