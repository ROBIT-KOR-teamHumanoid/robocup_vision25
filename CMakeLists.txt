cmake_minimum_required(VERSION 3.8)
project(robocup_vision25)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(humanoid_interfaces REQUIRED)
find_package(dynamixel_rdk_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(SOURCES
  src/robocup_vision25.cpp
)

set(HEADERS
  include/robocup_vision25/robocup_vision25.hpp
  include/YoloV4/YoloV4.hpp
  include/pan_tilt/pan_tilt.hpp
)

# Declare the executable
add_executable(robocup_vision25 ${SOURCES})

# Link libraries
target_link_libraries(robocup_vision25
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Boost::system
  Boost::thread
  -lv4l2
  -ludev
)

# Add dependencies
ament_target_dependencies(robocup_vision25
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  camera_info_manager
  OpenCV
  Boost
  humanoid_interfaces
  dynamixel_rdk_msgs
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

install(TARGETS
  robocup_vision25
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
