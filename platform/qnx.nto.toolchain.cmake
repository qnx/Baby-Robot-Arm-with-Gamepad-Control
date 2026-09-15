# Authoritative QNX toolchain for this repository.
#
# The compatibility file under cmake/ includes this file so the two historical
# entry points cannot drift in Python ABI or compiler policy.

if(BABY_ROBOT_QNX_TOOLCHAIN_INCLUDED)
  return()
endif()
set(BABY_ROBOT_QNX_TOOLCHAIN_INCLUDED TRUE)

# QCC and Python both consume ambient process controls before project code can
# defend itself.  The supported build also starts from an isolated environment;
# clear these here so direct configure invocations cannot redirect qcc's
# configuration or turn warning-category parsing into a host command launch.
unset(ENV{QCC_CONF_PATH})
unset(ENV{PYTHONWARNINGS})
unset(ENV{BROWSER})
unset(ENV{PYTHONHOME})
unset(ENV{PYTHONSTARTUP})
unset(ENV{PYTHONINSPECT})
unset(ENV{PYTHONPLATLIBDIR})
set(ENV{PYTHONNOUSERSITE} 1)
set(ENV{PYTHONSAFEPATH} 1)

if("$ENV{QNX_HOST}" STREQUAL "")
  message(FATAL_ERROR "QNX_HOST environment variable is required by the QNX toolchain")
endif()
if("$ENV{QNX_TARGET}" STREQUAL "")
  message(FATAL_ERROR "QNX_TARGET environment variable is required by the QNX toolchain")
endif()
if("$ENV{ARCH}" STREQUAL "" OR "$ENV{CPUVAR}" STREQUAL "" OR
   "$ENV{CPUVARDIR}" STREQUAL "")
  message(FATAL_ERROR "ARCH, CPUVAR, and CPUVARDIR are required by the QNX toolchain")
endif()

file(TO_CMAKE_PATH "$ENV{QNX_HOST}" QNX_HOST)
file(TO_CMAKE_PATH "$ENV{QNX_TARGET}" QNX_TARGET)
set(QNX_STAGE "$ENV{QNX_STAGE}")
set(ARCH "$ENV{ARCH}")
set(CPUVAR "$ENV{CPUVAR}")
set(CPUVARDIR "$ENV{CPUVARDIR}")

foreach(qnx_token ARCH CPUVAR CPUVARDIR)
  if(NOT "${${qnx_token}}" MATCHES "^[A-Za-z0-9_]+$")
    message(FATAL_ERROR "${qnx_token} contains characters unsafe for a compiler variant or path")
  endif()
endforeach()

foreach(required_path QNX_HOST QNX_TARGET)
  if(NOT "${${required_path}}" MATCHES "^/" OR NOT IS_DIRECTORY "${${required_path}}")
    message(FATAL_ERROR "${required_path} must resolve to an existing absolute directory")
  endif()
endforeach()

set(QNX_C_COMPILER "${QNX_HOST}/usr/bin/qcc")
set(QNX_CXX_COMPILER "${QNX_HOST}/usr/bin/q++")
if(NOT EXISTS "${QNX_C_COMPILER}" OR NOT EXISTS "${QNX_CXX_COMPILER}")
  message(FATAL_ERROR "QNX compiler drivers were not found beneath QNX_HOST")
endif()

message(STATUS "using QNX_HOST ${QNX_HOST}")
message(STATUS "using QNX_TARGET ${QNX_TARGET}")
message(STATUS "using CPUVAR ${CPUVAR}")
message(STATUS "using CPUVARDIR ${CPUVARDIR}")
message(STATUS "using ARCH ${ARCH}")

# Only repository-owned, explicitly located compatibility modules are added.
list(PREPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_LIST_DIR}/modules"
  "${CMAKE_CURRENT_LIST_DIR}/../cmake/modules")

set(QNX TRUE)
set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_PROCESSOR "${CPUVAR}")
set(CMAKE_C_COMPILER "${QNX_C_COMPILER}" CACHE FILEPATH "QNX C compiler" FORCE)
set(CMAKE_CXX_COMPILER "${QNX_CXX_COMPILER}" CACHE FILEPATH "QNX C++ compiler" FORCE)

list(APPEND CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES "${QNX_TARGET}/usr/include")

set(QNX_COMMON_COMPILE_FLAGS
  "-DTIXML_USE_STL -DOPENCV_NOSTL_TRANSITIONAL -D_QNX_SOURCE -D__USESRCVERSION")
set(QNX_RPATH_LINK_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
if(ROS_EXTERNAL_DEPS_INSTALL)
  if(NOT "${ROS_EXTERNAL_DEPS_INSTALL}" MATCHES "^/" OR
     NOT IS_DIRECTORY "${ROS_EXTERNAL_DEPS_INSTALL}")
    message(FATAL_ERROR "ROS_EXTERNAL_DEPS_INSTALL must be an existing absolute directory")
  endif()
  string(APPEND QNX_COMMON_COMPILE_FLAGS
    " -I${ROS_EXTERNAL_DEPS_INSTALL}/${CPUVARDIR}/include/foonathan/memory/detail")
  string(PREPEND QNX_RPATH_LINK_DIRS
    "${ROS_EXTERNAL_DEPS_INSTALL}/${CPUVARDIR}/usr/lib:")
endif()

# Do not suppress warning classes globally. Individual third-party targets may
# request a narrowly scoped suppression with an explanation where unavoidable.
string(APPEND CMAKE_C_FLAGS_INIT
  " -Vgcc_nto${CPUVAR} ${QNX_COMMON_COMPILE_FLAGS} -Wl,-rpath-link,${QNX_RPATH_LINK_DIRS}")
string(APPEND CMAKE_CXX_FLAGS_INIT
  " -Vgcc_nto${CPUVAR} ${QNX_COMMON_COMPILE_FLAGS} -Wl,-rpath-link,${QNX_RPATH_LINK_DIRS} -stdlib=libc++ -std=c++17")
string(APPEND CMAKE_EXE_LINKER_FLAGS_INIT " -Wl,--build-id=md5,--as-needed")
string(APPEND CMAKE_SHARED_LINKER_FLAGS_INIT " -Wl,--build-id=md5,--as-needed")
if(ROS_EXTERNAL_DEPS_INSTALL)
  string(APPEND CMAKE_EXE_LINKER_FLAGS_INIT " -L${ROS_EXTERNAL_DEPS_INSTALL}/lib")
  string(APPEND CMAKE_SHARED_LINKER_FLAGS_INIT " -L${ROS_EXTERNAL_DEPS_INSTALL}/lib")
endif()

if("${ARCH}" STREQUAL "arm")
  set(QNX_TOOL_PREFIX "${QNX_HOST}/usr/bin/ntoarmv7")
else()
  set(QNX_TOOL_PREFIX "${QNX_HOST}/usr/bin/nto${ARCH}")
endif()
set(CMAKE_AR "${QNX_TOOL_PREFIX}-ar${HOST_EXECUTABLE_SUFFIX}" CACHE FILEPATH "QNX archiver" FORCE)
set(CMAKE_RANLIB "${QNX_TOOL_PREFIX}-ranlib${HOST_EXECUTABLE_SUFFIX}" CACHE FILEPATH "QNX ranlib" FORCE)
set(CMAKE_STRIP "${QNX_TOOL_PREFIX}-strip${HOST_EXECUTABLE_SUFFIX}" CACHE FILEPATH "QNX strip" FORCE)

set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)

########################################################################
# Python 3.11 target ABI and exact host interpreter validation
########################################################################
unset(QNX_HOST_PYTHON_EXECUTABLE CACHE)
unset(QNX_HOST_PYTHON_EXECUTABLE)
find_program(QNX_HOST_PYTHON_EXECUTABLE
  NAMES python3.11
  PATHS
    "/usr/local/qnx/env/bin"
    "/usr/local/bin"
    "/usr/bin"
    "/bin"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)
if(NOT QNX_HOST_PYTHON_EXECUTABLE)
  message(FATAL_ERROR "An exact Python 3.11 host interpreter is required")
endif()
file(REAL_PATH "${QNX_HOST_PYTHON_EXECUTABLE}" QNX_HOST_PYTHON_EXECUTABLE_REAL)
if(NOT QNX_HOST_PYTHON_EXECUTABLE_REAL MATCHES
   "^/(usr/local/qnx/env/bin|usr/local/bin|usr/bin|bin)/python3\\.11$")
  message(FATAL_ERROR
    "Host Python must resolve beneath an administrator-managed executable directory")
endif()
set(QNX_HOST_PYTHON_EXECUTABLE "${QNX_HOST_PYTHON_EXECUTABLE_REAL}")
set(Python3_EXECUTABLE "${QNX_HOST_PYTHON_EXECUTABLE}" CACHE FILEPATH
  "Exact Python 3.11 host interpreter" FORCE)
set(PYTHON_EXECUTABLE "${QNX_HOST_PYTHON_EXECUTABLE}")
find_package(PythonInterp 3.11 REQUIRED)
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -I -c
    "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  RESULT_VARIABLE QNX_PYTHON_VERSION_RESULT
  OUTPUT_VARIABLE QNX_HOST_PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT QNX_PYTHON_VERSION_RESULT EQUAL 0 OR
   NOT QNX_HOST_PYTHON_VERSION STREQUAL "3.11")
  message(FATAL_ERROR
    "QNX cross-build requires an exact Python 3.11 host interpreter; found '${QNX_HOST_PYTHON_VERSION}'")
endif()

set(PYTHON_SOABI "cpython-311")
set(QNX_TARGET_PYTHON_INCLUDE_CANDIDATES
  "${QNX_TARGET}/usr/include/${CPUVARDIR}/python3.11"
  "${QNX_TARGET}/${CPUVARDIR}/usr/include/python3.11"
  "${QNX_TARGET}/usr/include/python3.11")
set(PYTHON_INCLUDE_DIRS)
foreach(python_include IN LISTS QNX_TARGET_PYTHON_INCLUDE_CANDIDATES)
  if(EXISTS "${python_include}/Python.h")
    list(APPEND PYTHON_INCLUDE_DIRS "${python_include}")
  endif()
endforeach()
if(NOT PYTHON_INCLUDE_DIRS)
  message(FATAL_ERROR "No target Python 3.11 include directory containing Python.h was found")
endif()

set(QNX_TARGET_NUMPY_INCLUDE_CANDIDATES
  "${QNX_TARGET}/${CPUVARDIR}/usr/lib/python3.11/site-packages/numpy/core/include")
if(ROS_EXTERNAL_DEPS_INSTALL)
  list(APPEND QNX_TARGET_NUMPY_INCLUDE_CANDIDATES
    "${ROS_EXTERNAL_DEPS_INSTALL}/${CPUVARDIR}/usr/lib/python3.11/site-packages/numpy/core/include")
endif()
foreach(numpy_include IN LISTS QNX_TARGET_NUMPY_INCLUDE_CANDIDATES)
  if(EXISTS "${numpy_include}/numpy/arrayobject.h")
    list(APPEND PYTHON_INCLUDE_DIRS "${numpy_include}")
  endif()
endforeach()
set(PYTHON_INCLUDE_DIR "${PYTHON_INCLUDE_DIRS}")
set(PYTHON_LIBRARY "${QNX_TARGET}/${CPUVARDIR}/usr/lib/libpython3.11.so")
if(NOT EXISTS "${PYTHON_LIBRARY}" OR IS_DIRECTORY "${PYTHON_LIBRARY}")
  message(FATAL_ERROR "Target Python 3.11 library was not found at '${PYTHON_LIBRARY}'")
endif()
set(PYTHON_LIBRARIES "${PYTHON_LIBRARY}")
set(PYTHONLIBS_FOUND TRUE)
set(PYTHON_MODULE_EXTENSION "cpython-311.so")
set(PYTHON_IS_DEBUG FALSE)

function(python3_add_library name)
  cmake_parse_arguments(PARSE_ARGV 1 PYTHON_ADD_LIBRARY
    "STATIC;SHARED;MODULE;WITH_SOABI" "" "")

  if(PYTHON_ADD_LIBRARY_STATIC)
    set(type STATIC)
  elseif(PYTHON_ADD_LIBRARY_SHARED)
    set(type SHARED)
  else()
    set(type MODULE)
  endif()

  add_library(${name} ${type} ${PYTHON_ADD_LIBRARY_UNPARSED_ARGUMENTS})
  target_include_directories(${name} PRIVATE ${PYTHON_INCLUDE_DIRS})
  target_link_libraries(${name} PRIVATE "${PYTHON_LIBRARY}")

  get_property(created_type TARGET ${name} PROPERTY TYPE)
  if(created_type STREQUAL "MODULE_LIBRARY")
    set_property(TARGET ${name} PROPERTY PREFIX "")
    if(PYTHON_ADD_LIBRARY_WITH_SOABI)
      set_property(TARGET ${name} PROPERTY SUFFIX ".${PYTHON_MODULE_EXTENSION}")
    endif()
  elseif(PYTHON_ADD_LIBRARY_WITH_SOABI)
    message(AUTHOR_WARNING "WITH_SOABI is supported only for MODULE libraries")
  endif()
endfunction()

if(ROS_EXTERNAL_DEPS_INSTALL)
  set(Eigen3_INCLUDE_DIRS "${ROS_EXTERNAL_DEPS_INSTALL}/include/eigen3")
else()
  set(Eigen3_INCLUDE_DIRS "${QNX_TARGET}/usr/include/eigen3")
endif()
set(EIGEN3_FOUND TRUE)

########################################################################
# Target dependency search roots
########################################################################
set(CMAKE_FIND_ROOT_PATH
  "${CMAKE_INSTALL_PREFIX}"
  "${QNX_TARGET}"
  "${QNX_TARGET}/${CPUVARDIR}")

if(ROS2_HOST_INSTALLATION_PATH)
  if(NOT "${ROS2_HOST_INSTALLATION_PATH}" MATCHES "^/" OR
     NOT IS_DIRECTORY "${ROS2_HOST_INSTALLATION_PATH}")
    message(FATAL_ERROR "ROS2_HOST_INSTALLATION_PATH must be an existing absolute directory")
  endif()
  list(APPEND CMAKE_FIND_ROOT_PATH "${ROS2_HOST_INSTALLATION_PATH}")
endif()

if(NOT "$ENV{NDDSHOME}" STREQUAL "")
  file(TO_CMAKE_PATH "$ENV{NDDSHOME}" NDDSHOME)
  if(NOT "${NDDSHOME}" MATCHES "^/" OR NOT IS_DIRECTORY "${NDDSHOME}")
    message(FATAL_ERROR "NDDSHOME must be an existing absolute directory")
  endif()
  list(APPEND CMAKE_FIND_ROOT_PATH "${NDDSHOME}")
endif()

if(ROS_EXTERNAL_DEPS_INSTALL)
  list(APPEND CMAKE_FIND_ROOT_PATH
    "${ROS_EXTERNAL_DEPS_INSTALL}"
    "${ROS_EXTERNAL_DEPS_INSTALL}/${CPUVARDIR}")
endif()

# Runtime paths differ on the target; do not embed host staging paths.
set(CMAKE_SKIP_RPATH TRUE CACHE BOOL "Do not embed host build paths" FORCE)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
