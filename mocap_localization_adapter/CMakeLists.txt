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
project(mocap_localization_adapter LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(localization_adapter_interfaces REQUIRED)
find_package(localization_contracts REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/adapter_config.cpp
  src/adapter_node.cpp
  src/pose_transform.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  localization_adapter_interfaces
  localization_contracts
  rcl_interfaces
  rclcpp
  rmw
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

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

  ament_add_gtest(test_pose_transform test/test_pose_transform.cpp)
  if(TARGET test_pose_transform)
    target_link_libraries(test_pose_transform ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_shadow_node_contract test/test_shadow_node_contract.cpp)
  if(TARGET test_shadow_node_contract)
    target_link_libraries(test_shadow_node_contract ${PROJECT_NAME}_core)
    ament_target_dependencies(test_shadow_node_contract rclcpp)
  endif()

  ament_add_gtest(test_live_shadow_pipeline test/test_live_shadow_pipeline.cpp)
  if(TARGET test_live_shadow_pipeline)
    target_link_libraries(test_live_shadow_pipeline ${PROJECT_NAME}_core)
    ament_target_dependencies(test_live_shadow_pipeline
      diagnostic_msgs
      geometry_msgs
      localization_adapter_interfaces
      rclcpp
    )
    set_tests_properties(test_live_shadow_pipeline PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=229;ROS_LOCALHOST_ONLY=1"
    )
  endif()

  ament_add_gtest(test_output_authority test/test_output_authority.cpp)
  if(TARGET test_output_authority)
    target_link_libraries(test_output_authority ${PROJECT_NAME}_core)
    ament_target_dependencies(test_output_authority
      geometry_msgs
      localization_adapter_interfaces
      rclcpp
    )
    set_tests_properties(test_output_authority PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=230;ROS_LOCALHOST_ONLY=1"
    )
  endif()

  ament_add_gtest(
    test_source_candidate_authority
    test/test_source_candidate_authority.cpp
  )
  if(TARGET test_source_candidate_authority)
    target_link_libraries(test_source_candidate_authority ${PROJECT_NAME}_core)
    ament_target_dependencies(test_source_candidate_authority
      diagnostic_msgs
      geometry_msgs
      localization_adapter_interfaces
      rclcpp
    )
    set_tests_properties(test_source_candidate_authority PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=225;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 20
    )
  endif()

  ament_add_gtest(
    test_input_authority_lifecycle
    test/test_input_authority_lifecycle.cpp
  )
  if(TARGET test_input_authority_lifecycle)
    target_link_libraries(test_input_authority_lifecycle ${PROJECT_NAME}_core)
    ament_target_dependencies(test_input_authority_lifecycle
      diagnostic_msgs
      geometry_msgs
      localization_adapter_interfaces
      rclcpp
    )
    set_tests_properties(test_input_authority_lifecycle PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=231;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 60
    )
  endif()

  ament_add_gtest(
    test_input_authority_timeout
    test/test_input_authority_timeout.cpp
  )
  if(TARGET test_input_authority_timeout)
    target_link_libraries(test_input_authority_timeout ${PROJECT_NAME}_core)
    ament_target_dependencies(test_input_authority_timeout
      diagnostic_msgs
      geometry_msgs
      localization_adapter_interfaces
      rclcpp
    )
    set_tests_properties(test_input_authority_timeout PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=232;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 20
    )
  endif()
endif()

ament_package()
