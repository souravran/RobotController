#
# component
#------------------------------------------------------------

set(motion_controller_SRCS
 PathExecuter.cpp
)

add_library(motion_controller SHARED ${motion_controller_SRCS})
target_link_libraries(motion_controller ${LOG4CPP_LIBRARIES})
#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
