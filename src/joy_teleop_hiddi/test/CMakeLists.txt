cmake_minimum_required(VERSION 3.10)
project(joy_teleop_parser_tests LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()
add_executable(parser_unit_tests
  test_parser.c
  ../src/parser.c
)
target_include_directories(parser_unit_tests PRIVATE
  stubs
  ../include
)
add_test(NAME parser_unit_tests COMMAND parser_unit_tests)

add_executable(publication_gate_tests test_publication_gate.cpp)
target_include_directories(publication_gate_tests PRIVATE ../include)
target_compile_features(publication_gate_tests PRIVATE cxx_std_17)
add_test(NAME publication_gate_tests COMMAND publication_gate_tests)
