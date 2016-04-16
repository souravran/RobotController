#
# component
#------------------------------------------------------------

set(map_management_SRCS
  Cell.cpp
  CellProperty.cpp
  MapProxyClient.cpp
  Map.cpp
)

add_library(map_management SHARED ${map_management_SRCS})
target_link_libraries(map_management ${LOG4CPP_LIBRARIES} ${Boost_LIBRARIES})
#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
