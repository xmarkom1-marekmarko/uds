cmake_minimum_required(VERSION 3.8)
project(zad3)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(states_from_keyboard_node src/statesFromKeyboard.cpp)
add_executable(states_node src/statesNode.cpp)

# Link dependencies
ament_target_dependencies(states_from_keyboard_node
  rclcpp
  std_msgs
)

ament_target_dependencies(states_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

# Install
install(TARGETS
  states_from_keyboard_node
  states_node
  DESTINATION lib/${PROJECT_NAME}
)

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



