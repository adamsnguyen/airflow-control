cmake_minimum_required(VERSION 3.5)
project(mqtt_ros_interface)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)

# MQTT mqtt_sub_ros_pub library
add_library(mqtt_sub_ros_pub_library STATIC src/mqtt_sub_ros_pub.cpp include/mqtt_sub_ros_pub.h)

target_include_directories(mqtt_sub_ros_pub_library PUBLIC include)

target_link_libraries(mqtt_sub_ros_pub_library PUBLIC paho-mqttpp3 paho-mqtt3as mqttHelper)
ament_target_dependencies(mqtt_sub_ros_pub_library PUBLIC rclcpp std_msgs)

install(TARGETS mqtt_sub_ros_pub_library
  DESTINATION lib/${PROJECT_NAME})

# MQTT Helper library
add_library(mqttHelper STATIC src/mqtt_helper.cpp include/mqtt_helper.h)

target_include_directories(mqttHelper PUBLIC include)

target_link_libraries(mqttHelper PUBLIC paho-mqttpp3 paho-mqtt3as mqtt_sub_ros_pub_library)

# mqtt_pub_ros_sub
add_executable(mqtt_pub_ros_sub src/mqtt_pub_ros_sub.cpp)

target_include_directories(mqtt_pub_ros_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(mqtt_pub_ros_sub PUBLIC paho-mqttpp3 paho-mqtt3as mqttHelper)

ament_target_dependencies(mqtt_pub_ros_sub PUBLIC rclcpp std_msgs)

install(TARGETS mqtt_pub_ros_sub
  DESTINATION lib/${PROJECT_NAME})

# mqtt_sub_ros_pub_exec
add_executable(mqtt_sub_ros_pub_exec src/mqtt_sub_ros_pub_exec.cpp)

target_include_directories(mqtt_sub_ros_pub_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(mqtt_sub_ros_pub_exec PUBLIC paho-mqttpp3 paho-mqtt3as mqttHelper mqtt_sub_ros_pub_library)

ament_target_dependencies(mqtt_sub_ros_pub_exec PUBLIC rclcpp std_msgs)

install(TARGETS mqtt_sub_ros_pub_exec
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
