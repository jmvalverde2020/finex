cmake_minimum_required(VERSION 3.8)
project(reflex-exo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lbcm2835)
endif()

list(APPEND CMAKE_PREFIX_PATH "/usr/include/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# include cpp "include" directory
include_directories(include)

# create cpp executables
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs)

# install cpp executables
install(TARGETS
    control_node
    DESTINATION lib/${PROJECT_NAME}
)

# install Python modules
#ament_python_install_package(${PROJECT_NAME})

# install Python executables
install(PROGRAMS
    scripts/sensor_node.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
