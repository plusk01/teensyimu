cmake_minimum_required(VERSION 3.10)
project(teensyimu)

set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

###############################################################################
# async_comm
###############################################################################

set(ASYNC_COMM_DIR "${CMAKE_CURRENT_BINARY_DIR}/async_comm-download" CACHE INTERNAL "async_comm build dir" FORCE)
configure_file(${CMAKE_MODULE_PATH}/async_comm.cmake.in ${ASYNC_COMM_DIR}/CMakeLists.txt IMMEDIATE @ONLY)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${ASYNC_COMM_DIR})
execute_process(COMMAND "${CMAKE_COMMAND}" --build . WORKING_DIRECTORY ${ASYNC_COMM_DIR})
add_subdirectory(${ASYNC_COMM_DIR}/src)

###############################################################################
# Targets
###############################################################################

add_library(acl_serial_driver SHARED src/serial_driver.cpp)
target_link_libraries(acl_serial_driver PRIVATE async_comm)
target_include_directories(acl_serial_driver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)


add_executable(screenprint src/screenprint.cpp)
target_link_libraries(screenprint acl_serial_driver)