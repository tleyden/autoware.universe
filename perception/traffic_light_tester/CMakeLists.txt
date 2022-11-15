cmake_minimum_required(VERSION 3.14)
project(traffic_light_tester)

find_package(autoware_cmake REQUIRED)
autoware_package()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(yaml-cpp REQUIRED)


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

ament_auto_add_library(traffic_light_tester_nodelet SHARED
  src/nodelet.cpp
)
target_link_libraries(traffic_light_tester_nodelet
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
rclcpp_components_register_node(traffic_light_tester_nodelet
  PLUGIN "traffic_light::TrafficLightTesterNodelet"
  EXECUTABLE traffic_light_tester_node
)

ament_auto_package(INSTALL_TO_SHARE
  data
  launch
)
