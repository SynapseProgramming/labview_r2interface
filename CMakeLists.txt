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
  "msg/Laserarray.msg"
 )

#add executables

add_executable(laserscan_convert src/laserscan_convert.cpp)
ament_target_dependencies(laserscan_convert rclcpp std_msgs sensor_msgs)
rosidl_target_interfaces(laserscan_convert ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(odom_convert src/odom_convert.cpp)
ament_target_dependencies(odom_convert rclcpp nav_msgs sensor_msgs tf2 tf2_ros)
rosidl_target_interfaces(odom_convert ${PROJECT_NAME} "rosidl_typesupport_cpp")


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



#key in name here, for ro2 to find the executable
install(TARGETS
  laserscan_convert
  odom_convert


  DESTINATION lib/${PROJECT_NAME})

ament_package()
