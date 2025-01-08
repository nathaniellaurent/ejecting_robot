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
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIR})





add_executable(camera_publisher src/camera_pub.cpp)

target_link_libraries(camera_publisher ${EIGEN3_LIBRARIES})
ament_target_dependencies(camera_publisher rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport Eigen3 OpenCV)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp std_msgs sensor_msgs)





install(TARGETS
    camera_publisher
    listener
    teleop
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)



ament_package()
