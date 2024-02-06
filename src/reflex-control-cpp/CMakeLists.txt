cmake_minimum_required(VERSION 3.8)
project(reflex-control-cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs)

# Install Cpp executables
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
