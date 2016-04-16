#
# component
#------------------------------------------------------------

set(robot_controller_SRCS
    RobotBehaviorController.cpp
)

add_library(robot_controller SHARED ${robot_controller_SRCS})
target_link_libraries(robot_controller ${LOG4CPP_LIBRARIES})
#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
