#
# component
#------------------------------------------------------------

set(path_planner_SRCS
 PathPlannerAStar.cpp
)

add_library(path_planner SHARED ${path_planner_SRCS})
target_link_libraries(path_planner ${LOG4CPP_LIBRARIES} ${Boost_LIBRARIES})
#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
