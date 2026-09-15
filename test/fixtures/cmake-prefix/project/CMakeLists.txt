cmake_minimum_required(VERSION 3.22)
project(baby_robot_prefix_fixture NONE)

find_package(BabyRobotFixture REQUIRED CONFIG)
if(NOT BABY_ROBOT_FIXTURE_TRUSTED)
  message(FATAL_ERROR "trusted explicit prefix was not selected")
endif()
