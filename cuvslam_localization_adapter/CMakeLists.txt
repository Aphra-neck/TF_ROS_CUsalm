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
project(cuvslam_localization_adapter LANGUAGES CXX)

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
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_package(localization_adapter_interfaces REQUIRED)
find_package(localization_contracts REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/adapter_node.cpp
  src/contract_config.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  ament_index_cpp
  builtin_interfaces
  diagnostic_msgs
  isaac_ros_visual_slam_interfaces
  localization_adapter_interfaces
  localization_contracts
  nav_msgs
  rcl_interfaces
  rclcpp
  rmw
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME}_core OpenSSL::Crypto yaml-cpp Eigen3::Eigen)

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

  ament_add_gtest(test_shadow_node_contract test/test_shadow_node_contract.cpp)
  if(TARGET test_shadow_node_contract)
    target_link_libraries(test_shadow_node_contract ${PROJECT_NAME}_core)
    ament_target_dependencies(test_shadow_node_contract rclcpp)
    target_compile_definitions(test_shadow_node_contract PRIVATE
      TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  endif()

  ament_add_gtest(test_live_source_candidate test/test_live_source_candidate.cpp)
  if(TARGET test_live_source_candidate)
    target_link_libraries(test_live_source_candidate ${PROJECT_NAME}_core)
    ament_target_dependencies(test_live_source_candidate
      diagnostic_msgs
      isaac_ros_visual_slam_interfaces
      localization_adapter_interfaces
      nav_msgs
      rclcpp
    )
    target_compile_definitions(test_live_source_candidate PRIVATE
      TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    set_tests_properties(test_live_source_candidate PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=223;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 20
    )
  endif()

  ament_add_gtest(
    test_input_publisher_authority
    test/test_input_publisher_authority.cpp
  )
  if(TARGET test_input_publisher_authority)
    target_link_libraries(test_input_publisher_authority ${PROJECT_NAME}_core)
    ament_target_dependencies(test_input_publisher_authority
      diagnostic_msgs
      isaac_ros_visual_slam_interfaces
      localization_adapter_interfaces
      nav_msgs
      rclcpp
    )
    target_compile_definitions(test_input_publisher_authority PRIVATE
      TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    set_tests_properties(test_input_publisher_authority PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=224;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 30
    )
  endif()
endif()

ament_package()
