// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef JOB_SCHEDULER_JOBSIMULATOR_H_
#define JOB_SCHEDULER_JOBSIMULATOR_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/IJobSimulator.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <list>

namespace accmetnavigation {
/*!
 */
class JobSimulator : public IJobSimulator {
 public:
  typedef std::shared_ptr<JobSimulator> Ptr;
  static Ptr Create(IMapServer::Ptr pMapProxyClient);
  virtual ~JobSimulator();

  virtual std::list<Job::Ptr> GetAllJobs();
  virtual Job::Ptr GetJob();
  virtual void SetJobCompleted(int pJobID);

 protected:
  JobSimulator(IMapServer::Ptr pMapProxyClient);

 private:
  log4cpp::Category& mLogger;
  IMapServer::Ptr mMapProxyClient;
  std::list<Job::Ptr> mJobsList;
  std::list<Job::Ptr> mJobsInProgress;
  std::list<Job::Ptr> mJobsCompleted;

  void LoadJobs();
};
} /* namespace accmetnavigation */
#endif  // JOB_SCHEDULER_JOBSIMULATOR_H_
