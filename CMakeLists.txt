cmake_minimum_required(VERSION 3.5)
project(gimbal_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# nlohmann_json is header-only on Ubuntu; detect but don't require
find_package(nlohmann_json QUIET)

# Prefer system-provided Paho MQTT via rosdep
find_path(PAHO_MQTT_C_INCLUDE_DIRS NAMES MQTTClient.h PATH_SUFFIXES paho mqtt paho-mqtt)
find_path(PAHO_MQTT_CPP_INCLUDE_DIRS NAMES mqtt/async_client.h)
find_library(PAHO_MQTT_C_LIBRARIES NAMES paho-mqtt3as paho-mqtt3a)
find_library(PAHO_MQTT_CPP_LIBRARIES NAMES paho-mqttpp3)

include_directories(
  include
  ${PAHO_MQTT_C_INCLUDE_DIRS}
  ${PAHO_MQTT_CPP_INCLUDE_DIRS}
)

add_executable(gimbal_control_node src/gimbal_control_node.cpp)
ament_target_dependencies(gimbal_control_node rclcpp)
target_link_libraries(gimbal_control_node
  ${PAHO_MQTT_CPP_LIBRARIES}
  ${PAHO_MQTT_C_LIBRARIES}
)

install(TARGETS gimbal_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()


