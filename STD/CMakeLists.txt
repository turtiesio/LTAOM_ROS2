cmake_minimum_required(VERSION 3.8)
project(std_loop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB QUIET)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

if(TBB_FOUND)
  include_directories(${TBB_INCLUDE_DIRS})
endif()

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  builtin_interfaces
  cv_bridge
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
)

add_library(loop_detection SHARED
  src/loop_detection.cpp
  src/std.cpp
  src/std_ba.cpp
)
target_link_libraries(loop_detection
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
if(TBB_FOUND)
  target_link_libraries(loop_detection TBB::tbb)
endif()
ament_target_dependencies(loop_detection ${dependencies})

# Create executable
add_executable(loop_detection_node
  src/loop_detection.cpp
  src/std.cpp
  src/std_ba.cpp
)
target_link_libraries(loop_detection_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
if(TBB_FOUND)
  target_link_libraries(loop_detection_node TBB::tbb)
endif()
ament_target_dependencies(loop_detection_node ${dependencies})

install(TARGETS
  loop_detection
  loop_detection_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(loop_detection)
ament_export_dependencies(${dependencies})

ament_package()
