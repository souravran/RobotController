// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/Job.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <utility>

namespace accmetnavigation {

Job::Ptr Job::Create(int pJobID) {
  Job::Ptr retVal = Job::Ptr(new Job(pJobID));
  return retVal;
}

Job::~Job() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

int Job::GetID() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mJobID;
}

void Job::SetID(int pJobID) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mJobID = pJobID;
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Job::LocationPtr Job::GetPickupLocation() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mPickupLocation;
}

void Job::SetPickupLocation(Cell pPickupLocation) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mPickupLocation->SetProperty<unsigned int>("ID", pPickupLocation.GetProperty<unsigned int>("ID", 0));
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Job::LocationPtr Job::GetDropLocation() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mDropLocation;
}

void Job::SetDropLocation(Cell pDropLocation) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mDropLocation->SetProperty<unsigned int>("ID", pDropLocation.GetProperty<unsigned int>("ID", 0));
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Job::Job(int pJobID)
    : mLogger(log4cpp::Category::getInstance("Job")),
      mJobID(pJobID),
      mPickupLocation(new Cell),
      mDropLocation(new Cell) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
