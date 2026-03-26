cmake_minimum_required(VERSION 3.8)
project(payload_estimator)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  std_msgs
  sensor_msgs
  Eigen3
  yaml-cpp
  tf2
  tf2_ros
  rclcpp_components
  std_srvs
)

add_library(payload_estimator_nodelet SHARED
  src/nodelet.cpp
)
set_target_properties(payload_estimator_nodelet PROPERTIES COMPILE_DEFINITIONS "COMPOSITION_BUILDING_DLL")

target_include_directories(payload_estimator_nodelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(payload_estimator_nodelet ${dependencies})
target_link_libraries(payload_estimator_nodelet ${YAML_CPP_LIBRARIES})

rclcpp_components_register_node(payload_estimator_nodelet
  PLUGIN "payload_estimator_nodelet::PayloadEstimatorNodelet"
  EXECUTABLE payload_estimator_nodelet_exec)

ament_export_libraries(payload_estimator_nodelet)
ament_export_dependencies(${dependencies})

install(TARGETS
  payload_estimator_nodelet
  payload_estimator_nodelet_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
