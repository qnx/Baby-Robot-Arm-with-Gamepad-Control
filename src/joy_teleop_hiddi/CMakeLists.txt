#
# Copyright (c) 2025, BlackBerry Limited. All rights reserved.
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
cmake_minimum_required(VERSION 3.18)
project(joy_teleop_hiddi)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
 
add_executable(joy_teleop_node
  src/joy_teleop_node.cpp
  src/parser.c
)

target_include_directories(joy_teleop_node PRIVATE include)
 
# Link against the QNX HIDDI library
target_link_libraries(joy_teleop_node
  hiddi
)
 
ament_target_dependencies(joy_teleop_node
  "builtin_interfaces"
  "rclcpp"
  "sensor_msgs"
)

# HID reports are untrusted bytes. Probe and apply hardening independently to
# the C parser and C++ ROS code, then probe link policy with the C++ driver used
# by this mixed-language executable.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/BabyRobotHardening.cmake")
baby_robot_enable_checked_hardening(
  joy_teleop_node LANGUAGES C CXX LINK_LANGUAGE CXX)

if(BUILD_TESTING)
  enable_testing()
  add_executable(parser_unit_tests
    test/test_parser.c
    src/parser.c
  )
  target_include_directories(parser_unit_tests PRIVATE include)
  add_test(NAME parser_unit_tests COMMAND parser_unit_tests)

  add_executable(publication_gate_tests test/test_publication_gate.cpp)
  target_include_directories(publication_gate_tests PRIVATE include)
  target_compile_features(publication_gate_tests PRIVATE cxx_std_17)
  add_test(NAME publication_gate_tests COMMAND publication_gate_tests)
endif()
 
install(TARGETS
  joy_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)
 
ament_package()
