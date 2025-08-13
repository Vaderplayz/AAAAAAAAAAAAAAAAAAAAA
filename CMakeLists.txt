cmake_minimum_required(VERSION 3.8)
project(px4_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
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

add_executable(offboard src/set_offboard.cpp)
ament_target_dependencies(offboard rclcpp mavros_msgs geometry_msgs)
install(TARGETS offboard DESTINATION lib/${PROJECT_NAME})


add_executable(arm2 src/arm2.cpp)
ament_target_dependencies(arm2 rclcpp mavros_msgs geometry_msgs)
install(TARGETS arm2 DESTINATION lib/${PROJECT_NAME})

add_executable(mis1 src/mission1.cpp)
ament_target_dependencies(mis1 rclcpp mavros_msgs geometry_msgs )
install(TARGETS mis1 DESTINATION lib/${PROJECT_NAME})

# add_executable(mis2 src/mission2.cpp)
# ament_target_dependencies(mis2 rclcpp mavros_msgs geometry_msgs )
# install(TARGETS mis2 DESTINATION lib/${PROJECT_NAME})

add_executable(OFFF src/OFFBOARD_NODE.cpp)
ament_target_dependencies(OFFF rclcpp mavros_msgs geometry_msgs )
install(TARGETS OFFF DESTINATION lib/${PROJECT_NAME})


add_executable(MISS src/MISSION.cpp)
ament_target_dependencies(MISS rclcpp mavros_msgs geometry_msgs sensor_msgs)
install(TARGETS MISS DESTINATION lib/${PROJECT_NAME})

add_executable(CIRCLE src/offboard_circle.cpp)
ament_target_dependencies(CIRCLE rclcpp mavros_msgs geometry_msgs sensor_msgs)
install(TARGETS CIRCLE DESTINATION lib/${PROJECT_NAME})

add_executable(offCIRCLE src/only_offboard_circle.cpp)
ament_target_dependencies(offCIRCLE rclcpp mavros_msgs geometry_msgs )
install(TARGETS offCIRCLE DESTINATION lib/${PROJECT_NAME})


add_executable(8 src/hinhso8.cpp)
ament_target_dependencies(8 rclcpp mavros_msgs geometry_msgs )
install(TARGETS 8 DESTINATION lib/${PROJECT_NAME})


add_executable(takeoff src/cathacanh.cpp)
ament_target_dependencies(takeoff rclcpp mavros_msgs geometry_msgs )
install(TARGETS takeoff DESTINATION lib/${PROJECT_NAME})


ament_package()
