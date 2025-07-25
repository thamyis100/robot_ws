cmake_minimum_required(VERSION 3.5)
project(robot)

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

# Force CMake to use the FindBoost module, not the shipped BoostConfig.cmake
set(Boost_NO_BOOST_CMAKE ON)

# (Only if you’re using a custom Boost install; skip for system Boost)
# set(BOOST_ROOT "/opt/your_boost_prefix" CACHE PATH "Boost root")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
# get_target_property(_boost_inc Boost::system INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS ">>> Boost::system INTERFACE_INCLUDE_DIRS = ${_boost_inc}")


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

add_executable(serial_node
  src/serial_node/serial_node.cpp
  src/serial_node/serial_handler.cpp
  src/serial_node/mecanum_kinematics.cpp
  src/serial_node/motor_commander.cpp
)

add_executable(controller_node
  src/controller_node/controller_node.cpp
)

target_include_directories(serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(serial_node
  rclcpp
  std_msgs
  geometry_msgs
  Boost
)
ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  serial_node
  controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
