cmake_minimum_required(VERSION 3.8)
project(scenic_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)

find_package(glider REQUIRED)
find_package(scenic REQUIRED)
find_package(scenic_msgs REQUIRED)

set(node_plugins "")

include_directories(
    include 
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/scenic_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  gtsam
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  glider::glider
  scenic::scenic
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  scenic_msgs
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  rclcpp::rclcpp
  ${PROJECT_NAME}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "ScenicROS::ScenicNode")
set(node_plugins "${node_plugins}ScenicROS::ScenicNode;$<TARGET_FILE:${PROJECT_NAME}>\n")

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_ros)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  Eigen3
)

ament_package()
