#
# main
#------------------------------------------------------------
set(NavigationApp_SRCS
    NavigationApp.cpp
)
add_executable(AccmetNavigationApp ${NavigationApp_SRCS})
target_link_libraries(AccmetNavigationApp ${LOG4CPP_LIBRARIES} ${Boost_LIBRARIES} map_management robot_platform job_scheduler path_planner motion_controller robot_controller )


#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
