cmake_minimum_required(VERSION 3.5)
project(gnss_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include C++ headers
include_directories(include)

# Declare a C++ executable
add_executable(gnss_path_generate src/gnss_path_generate.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(gnss_path_generate
  rclcpp
  nav_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  gnss_path_generate
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()

