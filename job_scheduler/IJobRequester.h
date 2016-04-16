// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef JOB_SCHEDULER_IJOBREQUESTER_H_
#define JOB_SCHEDULER_IJOBREQUESTER_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/Job.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <memory>

namespace accmetnavigation {
/*!
 */
class IJobRequester {
 public:
  typedef std::shared_ptr<IJobRequester> Ptr;
  virtual ~IJobRequester() {}

  virtual Job::Ptr RequestJob() = 0;
  virtual void JobCompleted(int pJobID) = 0;
};

}  // namespace accmetnavigation
#endif  // JOB_SCHEDULER_IJOBREQUESTER_H_
