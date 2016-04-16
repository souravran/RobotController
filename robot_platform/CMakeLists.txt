#
# component
#------------------------------------------------------------

set(robot_platform_SRCS
  RobotPlatformController.cpp
  RobotPlatformSim.cpp
)

add_library(robot_platform SHARED ${robot_platform_SRCS})
target_link_libraries(robot_platform ${LOG4CPP_LIBRARIES})
#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
