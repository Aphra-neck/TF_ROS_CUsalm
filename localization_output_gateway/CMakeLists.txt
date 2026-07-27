# Copyright 2026 u5-4
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22.1)
project(localization_output_gateway LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_adapter_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/contract_config.cpp
  src/gateway_node.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  localization_adapter_interfaces
  mavros_msgs
  rcl_interfaces
  rclcpp
  rmw
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  foreach(profile cuvslam_primary mocap_primary)
    set(test_target test_${profile}_pose_output)
    ament_add_gtest(${test_target} test/test_active_pose_output.cpp)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} ${PROJECT_NAME}_core)
      ament_target_dependencies(${test_target}
        diagnostic_msgs
        geometry_msgs
        localization_adapter_interfaces
        mavros_msgs
        rclcpp
      )
      target_compile_definitions(${test_target} PRIVATE
        TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
        TEST_PROFILE="${profile}")
    endif()
  endforeach()

  add_launch_test(
    test/test_disabled_launch.py
    TARGET test_disabled_launch
    TIMEOUT 30
  )
  set_tests_properties(test_disabled_launch PROPERTIES
    ENVIRONMENT
      "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=222;ROS_LOCALHOST_ONLY=1"
  )
  set_tests_properties(test_cuvslam_primary_pose_output PROPERTIES
    ENVIRONMENT
      "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=221;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 30
  )
  set_tests_properties(test_mocap_primary_pose_output PROPERTIES
    ENVIRONMENT
      "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=223;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 30
  )
endif()

ament_package()
