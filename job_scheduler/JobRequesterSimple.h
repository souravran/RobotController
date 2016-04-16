// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef JOB_SCHEDULER_JOBREQUESTERSIMPLE_H_
#define JOB_SCHEDULER_JOBREQUESTERSIMPLE_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <job_scheduler/IJobRequester.h>
#include <job_scheduler/IJobSimulator.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>

namespace accmetnavigation {
/*!
 */
class JobRequesterSimple : public IJobRequester {
 public:
  typedef std::shared_ptr<JobRequesterSimple> Ptr;
  static Ptr Create(IJobSimulator::Ptr pJobSimulator);
  virtual ~JobRequesterSimple();

  virtual Job::Ptr RequestJob();
  virtual void JobCompleted(int pJobID);

 protected:
  explicit JobRequesterSimple(IJobSimulator::Ptr pJobSimulator);

 private:
  log4cpp::Category& mLogger;
  IJobSimulator::Ptr mJobSimulator;
};

} /* namespace accmetnavigation */
#endif  // JOB_SCHEDULER_JOBREQUESTERSIMPLE_H_
