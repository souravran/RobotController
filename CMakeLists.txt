cmake_minimum_required(VERSION 2.8)
project(AccmetNavigation-Application)

include(cmake/common.cmake)
include(cmake/lib/c++11.cmake)

pkg_check_modules(LOG4CPP REQUIRED log4cpp)
find_package(Boost COMPONENTS date_time filesystem graph regex system thread iostreams)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${LOG4CPP_INCLUDE_DIRS}
)

link_directories(
    ${Boost_LIBRARY_DIRS}
    ${LOG4CPP_LIBRARY_DIRS}
)

if(NOT EXISTS "${CMAKE_BINARY_DIR}/MapGraph.dot")
     configure_file(${CMAKE_SOURCE_DIR}/MapGraph.dot ${CMAKE_BINARY_DIR}/MapGraph.dot)
endif()


add_subdirectory(application)
add_subdirectory(map_management)
add_subdirectory(robot_platform)
add_subdirectory(job_scheduler)
add_subdirectory(path_planner)
add_subdirectory(motion_controller)
add_subdirectory(domain)
add_subdirectory(system_platform)
