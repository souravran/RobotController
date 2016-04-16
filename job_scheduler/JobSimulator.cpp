// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/JobSimulator.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <list>
#include <utility>

namespace accmetnavigation {

JobSimulator::Ptr JobSimulator::Create() {
  JobSimulator::Ptr retVal = JobSimulator::Ptr(new JobSimulator());
  return retVal;
}

JobSimulator::~JobSimulator() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

std::list<Job::Ptr> JobSimulator::GetAllJobs() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mJobsList;
}

Job::Ptr JobSimulator::GetJob() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  // give away the last job from the job pool and also put this job into job in progress list
  Job::Ptr pendingJob;
  if (!mJobsList.empty()) {
    pendingJob = mJobsList.back();
    mJobsList.pop_back();
    mJobsInProgress.push_back(pendingJob);
  } else {
    pendingJob = 0;
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Currently job pool is empty";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return pendingJob;
}

void JobSimulator::SetJobCompleted(int pJobID) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // go through the list of job in progress and transfer job with given ID from in progress to completed list
  for (auto completedJob : mJobsInProgress) {
    if (completedJob->GetID() == pJobID) {
      mJobsCompleted.push_back(completedJob);
      mJobsInProgress.remove(completedJob);
    }
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

JobSimulator::JobSimulator()
    : mLogger(log4cpp::Category::getInstance("JobSimulator")), mJobsList(), mJobsInProgress(), mJobsCompleted() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // set arbitrary chosen couple of cells on the track-layout from simulation model as pickup and drop cells for a job,
  // the cell IDs are provided in order to identify the Cell while path planning
  Cell pickupLocation;
  pickupLocation.SetProperty<unsigned int>("ID", 78);
  Cell dropLocation;
  dropLocation.SetProperty<unsigned int>("ID", 194);

  // create a test job with a dummy ID and making the job available in the job list
  Job::Ptr testJob = Job::Create(1);
  testJob->SetPickupLocation(pickupLocation);
  testJob->SetDropLocation(dropLocation);

  mJobsList.push_back(testJob);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
