cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(uars_ros_interface REQUIRED)

include_directories(include)

# Add executables
add_executable(offboard_flier src/offboard_control.cpp)
ament_target_dependencies(offboard_flier rclcpp std_srvs Eigen3 std_msgs geometry_msgs uars_ros_interface)
target_link_libraries(offboard_flier
    MAVSDK::mavsdk
)

install(TARGETS offboard_flier DESTINATION lib/${PROJECT_NAME})


# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
