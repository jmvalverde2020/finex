cmake_minimum_required(VERSION 3.8)
project(finex_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# include cpp "include" directory
include_directories(include)

###########
## Build ##
###########

add_executable(control_panel
  src/interface_node.cpp
  src/${PROJECT_NAME}/FinexInterface.cpp
  include/${PROJECT_NAME}/FinexInterface.hpp
)

target_include_directories(control_panel
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
  control_panel)

ament_target_dependencies(control_panel 
  rclcpp)

target_link_libraries(control_panel
  Qt${QT_VERSION_MAJOR}::Widgets
  )

#############
## Install ##
#############

# Install executables
install(TARGETS
    control_panel
    DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   add_subdirectory(test)

#   # ROS2 linters, but disable copyright test. PickNik's copyright's may not conform
#   # to this test
#   # set(ament_cmake_copyright_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
