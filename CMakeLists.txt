cmake_minimum_required(VERSION 3.10)
project(teensyimu VERSION 0.2)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

###############################################################################
# Options
###############################################################################

option(BUILD_SHARED_LIBS "Build shared library" ON)
option(BUILD_BINDINGS_PYTHON "Build Python bindings" ON)

###############################################################################
# async_comm
###############################################################################

set(ASYNC_COMM_DIR "${CMAKE_CURRENT_BINARY_DIR}/async_comm-download" CACHE INTERNAL "async_comm build dir" FORCE)
configure_file(${CMAKE_MODULE_PATH}/async_comm.cmake.in ${ASYNC_COMM_DIR}/CMakeLists.txt IMMEDIATE @ONLY)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${ASYNC_COMM_DIR})
execute_process(COMMAND "${CMAKE_COMMAND}" --build . WORKING_DIRECTORY ${ASYNC_COMM_DIR})
add_subdirectory(${ASYNC_COMM_DIR}/src)

###############################################################################
# Pybind11
###############################################################################

if(BUILD_BINDINGS_PYTHON)
  set(PYBIND11_DIR "${CMAKE_CURRENT_BINARY_DIR}/pybind11-download" CACHE INTERNAL "pybind11 build dir" FORCE)
  configure_file(${CMAKE_MODULE_PATH}/pybind11.cmake.in ${PYBIND11_DIR}/CMakeLists.txt IMMEDIATE @ONLY)
  execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${PYBIND11_DIR})
  execute_process(COMMAND "${CMAKE_COMMAND}" --build . WORKING_DIRECTORY ${PYBIND11_DIR})
  add_subdirectory(${PYBIND11_DIR}/src)

  add_subdirectory(bindings/python)
endif()

###############################################################################
# Targets
###############################################################################

add_library(teensyimu src/serial_driver.cpp)
set_target_properties(teensyimu PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION})
target_link_libraries(teensyimu PRIVATE async_comm)
target_include_directories(teensyimu PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_definitions(teensyimu PUBLIC PROJECT_VERSION="${PROJECT_VERSION}")


add_executable(screenprint src/screenprint.cpp)
target_link_libraries(screenprint teensyimu)