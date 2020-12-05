cmake_minimum_required(VERSION 3.5)
project(ken_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_executable(ken_controller
  src/ken_control_node.cpp src/ken_interface.cpp src/ken_driver.cpp)

ament_target_dependencies(ken_controller
    controller_interface
    controller_manager
    dynamixel_sdk
    hardware_interface
    rclcpp
)

install(TARGETS ken_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(ken_goal_checker
  src/ken_goal_checker_node.cpp)

ament_target_dependencies(ken_goal_checker
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
)

install(TARGETS ken_goal_checker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

install(DIRECTORY launch config sounds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
