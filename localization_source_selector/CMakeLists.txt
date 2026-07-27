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
project(localization_source_selector LANGUAGES CXX)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_adapter_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/contract_config.cpp
  src/selector_node.cpp
  src/yaw_alignment.cpp
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
  rcl_interfaces
  rclcpp
  rmw
  std_msgs
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen yaml-cpp)

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
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_contract_config test/test_contract_config.cpp)
  if(TARGET test_contract_config)
    target_link_libraries(test_contract_config ${PROJECT_NAME}_core)
    target_compile_definitions(test_contract_config PRIVATE
      TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
      TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endif()

  ament_add_gtest(test_yaw_alignment test/test_yaw_alignment.cpp)
  if(TARGET test_yaw_alignment)
    target_link_libraries(test_yaw_alignment ${PROJECT_NAME}_core)
  endif()

  foreach(test_name IN ITEMS
      test_selector_node_contract
      test_selector_authority
      test_selector_lifecycle
      test_selector_pose_semantics)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} ${PROJECT_NAME}_core)
      ament_target_dependencies(${test_name}
        diagnostic_msgs
        localization_adapter_interfaces
        rclcpp
      )
      target_compile_definitions(${test_name} PRIVATE
        TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
        TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()
  endforeach()

  set_tests_properties(test_selector_authority PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=226;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 30)
  set_tests_properties(test_selector_lifecycle PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=227;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 30)
  set_tests_properties(test_selector_node_contract PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=228;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 30)
  set_tests_properties(test_selector_pose_semantics PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=220;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 30)
endif()

ament_package()
