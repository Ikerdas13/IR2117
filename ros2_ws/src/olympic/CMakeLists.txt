cmake_minimum_required(VERSION 3.8)
project(olympic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(olympic_interfaces REQUIRED)


add_executable(rings_action_server src/rings_action_server.cpp)
ament_target_dependencies(rings_action_server
  rclcpp
  rclcpp_action
  olympic_interfaces
)


add_executable(rings_action_client src/rings_action_client.cpp)
ament_target_dependencies(rings_action_client
  rclcpp
  rclcpp_action
  olympic_interfaces
)


install(TARGETS
  rings_action_server
  rings_action_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

