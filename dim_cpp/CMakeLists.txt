message(STATUS "Processing CMakeLists.txt for dim_cpp")

cmake_minimum_required(VERSION 3.5)
project(dim_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "---------------------------------------------------------------------")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_planner REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${xarm_planner_INCLUDE_DIRS})
message(STATUS "xarm_planner include dirs: ${xarm_planner_INCLUDE_DIRS}")

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

add_executable(moveit_converter src/moveit_converter.cpp)
target_include_directories(moveit_converter PUBLIC
  ${xarm_planner_INCLUDE_DIRS}
)
ament_target_dependencies(moveit_converter
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  geometry_msgs
  moveit_msgs
  Eigen3
  graph_msgs
  pluginlib
  xarm_msgs
  xarm_planner
  moveit
)

install(TARGETS
  moveit_converter
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files, if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
