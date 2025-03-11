	
cmake_minimum_required(VERSION 3.5) 
project(robot_trajectory)

# Default to C99
if(NOT CMAKE_C_STANDARD) 
  set(CMAKE_C_STANDARD 99)
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set (CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package (ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(square src/square.cpp) 
ament_target_dependencies(square rclcpp geometry_msgs)
add_executable(square_odom src/square_odom.cpp) 
ament_target_dependencies(square_odom rclcpp nav_msgs)

install(TARGETS
	square_odom
	square
	DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
