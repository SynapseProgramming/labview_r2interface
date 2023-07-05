cmake_minimum_required(VERSION 3.5)
project(labview_r2interface)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Lvodom.msg"
 )


# link against interface
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

#add executables

add_executable(odom_convert src/odom_convert.cpp)
ament_target_dependencies(odom_convert rclcpp nav_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(odom_convert "${cpp_typesupport_target}")

add_executable(laser_filter src/laser_filter.cpp)
ament_target_dependencies(laser_filter rclcpp nav_msgs sensor_msgs)
target_link_libraries(laser_filter "${cpp_typesupport_target}")


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



#key in name here, for ros2 to find the executable
install(
  TARGETS
  odom_convert
  laser_filter
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY
  launch
  
  DESTINATION share/${PROJECT_NAME})


ament_package()
