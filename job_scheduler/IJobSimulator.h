// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef JOB_SCHEDULER_IJOBSIMULATOR_H_
#define JOB_SCHEDULER_IJOBSIMULATOR_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/Job.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <memory>
#include <list>

namespace accmetnavigation {
/*!
 */
class IJobSimulator {
 public:
  typedef std::shared_ptr<IJobSimulator> Ptr;
  virtual ~IJobSimulator() {}

  virtual std::list<Job::Ptr> GetAllJobs() = 0;
  virtual Job::Ptr GetJob() = 0;
  virtual void SetJobCompleted(int pJobID) = 0;
};
}  // namespace accmetnavigation
#endif  // JOB_SCHEDULER_IJOBSIMULATOR_H_
