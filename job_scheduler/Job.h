// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef JOB_SCHEDULER_JOB_H_
#define JOB_SCHEDULER_JOB_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/Cell.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <memory>
#include <utility>

namespace accmetnavigation {
/*!
 */
class Job {
 public:
  typedef std::shared_ptr<Job> Ptr;
  typedef std::shared_ptr<Cell> LocationPtr;

  static Ptr Create(int pJobID);
  virtual ~Job();

  int GetID();
  void SetID(int pJobID);
  Job::LocationPtr GetPickupLocation();
  void SetPickupLocation(Cell pPickupLocation);
  Job::LocationPtr GetDropLocation();
  void SetDropLocation(Cell pDropLocation);

 protected:
  explicit Job(int pJobID);

 private:
  log4cpp::Category& mLogger;
  int mJobID;
  LocationPtr mPickupLocation;
  LocationPtr mDropLocation;
};

} /* namespace accmetnavigation */
#endif  // JOB_SCHEDULER_JOB_H_
