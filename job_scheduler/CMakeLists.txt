#
# component
#------------------------------------------------------------

set(job_scheduler_SRCS
  Job.cpp
  JobSimulator.cpp
  JobRequesterSimple.cpp
)

add_library(job_scheduler SHARED ${job_scheduler_SRCS})
target_link_libraries(job_scheduler ${LOG4CPP_LIBRARIES})
#-------------------------------------------------------------------------------

#
# add directories
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
