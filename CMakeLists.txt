cmake_minimum_required(VERSION 3.8)
project(kinect_v2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-ggdb)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)

find_package(freenect2 REQUIRED)

# Include directories
include_directories($ENV{HOME}/freenect2/include)
include_directories(include)

# Create an executable
add_executable(kinect2_node src/kinect2_driver.cpp)

# Specify dependencies of the executable
ament_target_dependencies(kinect2_node rclcpp sensor_msgs freenect2)

# Link libraries
target_link_libraries(kinect2_node  ${freenect2_LIBRARIES})

# Install executable
install(TARGETS  
        kinect2_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
