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
project(bag_contract_probe LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE GIT_REVISION_RESULT
  OUTPUT_VARIABLE BAG_CONTRACT_PROBE_SOURCE_REVISION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT GIT_REVISION_RESULT EQUAL 0)
  set(BAG_CONTRACT_PROBE_SOURCE_REVISION "unknown")
endif()
execute_process(
  COMMAND git status --porcelain --untracked-files=no
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_DIRTY_STATE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(GIT_DIRTY_STATE)
  string(APPEND BAG_CONTRACT_PROBE_SOURCE_REVISION "-dirty")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_package(localization_adapter_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/analyzer.cpp
  src/audit.cpp
  src/statistics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  isaac_ros_visual_slam_interfaces
  localization_adapter_interfaces
  mavros_msgs
  nav_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  _GNU_SOURCE
  BAG_CONTRACT_PROBE_SOURCE_REVISION="${BAG_CONTRACT_PROBE_SOURCE_REVISION}"
)

add_executable(dual_shadow_probe src/main.cpp)
target_link_libraries(dual_shadow_probe ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME} dual_shadow_probe
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  isaac_ros_visual_slam_interfaces
  localization_adapter_interfaces
  mavros_msgs
  nav_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  yaml_cpp_vendor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_statistics test/test_statistics.cpp)
  if(TARGET test_statistics)
    target_link_libraries(test_statistics ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_audit test/test_audit.cpp)
  if(TARGET test_audit)
    target_include_directories(test_audit PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_audit ${PROJECT_NAME})
    ament_target_dependencies(test_audit
      diagnostic_msgs
      geometry_msgs
      isaac_ros_visual_slam_interfaces
      localization_adapter_interfaces
      mavros_msgs
      nav_msgs
      sensor_msgs
    )
  endif()
endif()

ament_package()
