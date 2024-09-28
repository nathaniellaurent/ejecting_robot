cmake_minimum_required(VERSION 3.8)
project(ejecting_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
