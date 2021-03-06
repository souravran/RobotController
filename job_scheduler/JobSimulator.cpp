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

//static const std::vector<std::pair<int,int>> JobListCellIDs = {{190, 123}, {334, 259}, {341, 265}, {402, 410}, {182, 308}, {122, 115}};
//static const std::vector<std::pair<int,int>> JobListCellIDs = {{182, 308}, {122, 115}};
//static const std::vector<std::pair<int,int>> JobListCellIDs = {{182, 326}, {190, 334}, {182, 326}, {190, 334}, {182, 326}, {190, 334}};
static const std::vector<std::pair<int,int>> JobListCellIDs = {{190, 334}, {182, 326}, {190, 334}, {182, 326}, {182, 326}, {190, 334}};

JobSimulator::Ptr JobSimulator::Create(IMapServer::Ptr pMapProxyClient) {
  JobSimulator::Ptr retVal = JobSimulator::Ptr(new JobSimulator(pMapProxyClient));
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

void JobSimulator::LoadJobs() {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    int jobID = 1;
    for(auto cellID : JobListCellIDs) {
    	mLogger << log4cpp::Priority::DEBUG << __func__ << ": The cell IDs are : "<<cellID.first << " -- "<<cellID.second;
        Cell::CellPtr pickupLocation = Cell::CellPtr(new Cell());
        pickupLocation = mMapProxyClient->GetCellByID(cellID.first);
        Cell::CellPtr dropLocation = Cell::CellPtr(new Cell());
        dropLocation = mMapProxyClient->GetCellByID(cellID.second);
        Job::Ptr todoJob = Job::Create(jobID);
        todoJob->SetPickupLocation(pickupLocation);
        todoJob->SetDropLocation(dropLocation);
        mJobsList.push_back(todoJob);
        jobID++;
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

JobSimulator::JobSimulator(IMapServer::Ptr pMapProxyClient)
: mLogger(log4cpp::Category::getInstance("JobSimulator"))
, mMapProxyClient(pMapProxyClient)
, mJobsList()
, mJobsInProgress()
, mJobsCompleted() {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // set arbitrary chosen couple of cells on the track-layout from simulation model as pickup and drop cells for a job,
  // the cell IDs are provided in order to identify the Cell while path planning
//  Cell pickupLocation;

//  Cell::CellPtr pickupLocation2 = Cell::CellPtr(new Cell());
//  pickupLocation2->SetProperty<unsigned int>("ID", 120);
//  Cell::CellPtr dropLocation2 = Cell::CellPtr(new Cell());
//  dropLocation2->SetProperty<unsigned int>("ID", 323);
//
//  Job::Ptr testJobAnother = Job::Create(2);
//  testJobAnother->SetPickupLocation(pickupLocation2);
//  testJobAnother->SetDropLocation(dropLocation2);
//  mJobsList.push_back(testJobAnother);
//
//  Cell::CellPtr pickupLocation = Cell::CellPtr(new Cell());
//  pickupLocation->SetProperty<unsigned int>("ID", 266);
//  Cell::CellPtr dropLocation = Cell::CellPtr(new Cell());
//  dropLocation->SetProperty<unsigned int>("ID", 208);
//
//  // create a test job with a dummy ID and making the job available in the job list
//  Job::Ptr testJob = Job::Create(1);
//  testJob->SetPickupLocation(pickupLocation);
//  testJob->SetDropLocation(dropLocation);
//  mJobsList.push_back(testJob);
    LoadJobs();
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
