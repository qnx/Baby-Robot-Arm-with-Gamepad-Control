#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
cmake_minimum_required(VERSION 3.18)
project(ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ik_solver_node
  src/ik_solver_node.cpp
)

target_include_directories(ik_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ik_solver_node PUBLIC cxx_std_17)

if(TARGET Eigen3::Eigen)
  target_link_libraries(ik_solver_node PRIVATE Eigen3::Eigen)
elseif(Eigen3_INCLUDE_DIRS)
  target_include_directories(ik_solver_node PRIVATE ${Eigen3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Eigen3 was found without an imported target or include directory")
endif()

ament_target_dependencies(ik_solver_node
  "ament_index_cpp"
  "rclcpp"
  "rcl_interfaces"
  "sensor_msgs"
  "kdl_parser"
  "std_msgs"
  "urdf"
  "orocos_kdl"
)

# Use one policy for every native command-processing executable. The helper
# probes the active QNX/host compiler and linker instead of assuming GNU flags.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/BabyRobotHardening.cmake")
baby_robot_enable_checked_hardening(
  ik_solver_node LANGUAGES CXX LINK_LANGUAGE CXX)

if(BUILD_TESTING)
  enable_testing()
  add_executable(safety_validation_test test/test_safety_validation.cpp)
  target_include_directories(safety_validation_test PRIVATE include)
  target_compile_features(safety_validation_test PRIVATE cxx_std_17)
  add_test(NAME safety_validation_test COMMAND safety_validation_test)
endif()

# ===================================================================
# Install
# ===================================================================

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ik_solver_node DESTINATION lib/${PROJECT_NAME})

ament_package()
