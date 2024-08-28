cmake_minimum_required(VERSION 3.20)
project(teensyimu VERSION 0.2)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

include(FetchContent)

###############################################################################
# Options
###############################################################################

option(BUILD_SHARED_LIBS "Build shared library" ON)
option(BUILD_BINDINGS_PYTHON "Build Python bindings" ON)

###############################################################################
# async_comm
###############################################################################

FetchContent_Declare(async_comm
  GIT_REPOSITORY  https://github.com/plusk01/async_comm
  GIT_TAG         msvc-fixes
)
FetchContent_MakeAvailable(async_comm)

###############################################################################
# Pybind11
###############################################################################

if(BUILD_BINDINGS_PYTHON)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY  https://github.com/pybind/pybind11
    GIT_TAG         v2.13.5
  )
  FetchContent_MakeAvailable(pybind11)

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