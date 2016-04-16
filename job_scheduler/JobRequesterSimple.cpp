// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/JobRequesterSimple.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <list>

namespace accmetnavigation {

JobRequesterSimple::Ptr JobRequesterSimple::Create(IJobSimulator::Ptr pJobSimulator) {
  JobRequesterSimple::Ptr retVal = JobRequesterSimple::Ptr(new JobRequesterSimple(pJobSimulator));
  return retVal;
}

JobRequesterSimple::~JobRequesterSimple() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Job::Ptr JobRequesterSimple::RequestJob() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  Job::Ptr retVal;
  retVal = mJobSimulator->GetJob();

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return retVal;
}

void JobRequesterSimple::JobCompleted(int pJobID) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

JobRequesterSimple::JobRequesterSimple(IJobSimulator::Ptr pJobSimulator)
    : mLogger(log4cpp::Category::getInstance("JobRequesterSimple")), mJobSimulator(pJobSimulator) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
