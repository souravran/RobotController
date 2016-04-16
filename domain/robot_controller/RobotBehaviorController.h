// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef DOMAIN_ROBOT_CONTROLLER_ROBOTBEHAVIORCONTROLLER_H_
#define DOMAIN_ROBOT_CONTROLLER_ROBOTBEHAVIORCONTROLLER_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <robot_platform/IRobotPlatform.h>
#include <job_scheduler/IJobRequester.h>
#include <motion_controller/IPathExecuter.h>
#include <path_planner/IPathPlanner.h>
#include <map_management/Cell.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <memory>
#include <utility>

namespace accmetnavigation {

/*! class for robot behavior control.
 *  The class is used for logically coordinate among other packages to execute robot sub-behaviors.
 *  It is the main controlling unit of the system .
 */
class RobotBehaviorController {
 public:
  typedef void (RobotBehaviorController::*State)();
  typedef std::shared_ptr<RobotBehaviorController> Ptr;

  static Ptr Create(IRobotPlatform::Ptr pRobotPlatform, IPathExecuter::Ptr pMotionExecuter,
                    IJobRequester::Ptr pJobRequester, IPathPlanner::Ptr pPathPlanner);
  virtual ~RobotBehaviorController();

  /*! The method used for updating the behavioral states of the robot.
   *  This method gets called repeatedly and execute current robot-behavioral state.
   */
  void Update();

 protected:
  RobotBehaviorController(IRobotPlatform::Ptr pRobotPlatform, IPathExecuter::Ptr pMotionExecuter,
                          IJobRequester::Ptr pJobRequester, IPathPlanner::Ptr pPathPlanner);

 private:
  log4cpp::Category &mLogger;          //!< For logging messages to the console
  IRobotPlatform::Ptr mRobotPlatform;  //!< Platform for handling robot operations
  IPathExecuter::Ptr mPathExecuter;
  IJobRequester::Ptr mJobRequesterSimple;
  IPathPlanner::Ptr mPathPlanner;
  Job::Ptr mCurrentJob;
  State mState;                         //!< The behavioral state of the robot
  boost::posix_time::ptime mStartTime;  //!< Used to track time for any waiting-state operation
  IMapServer::CellPtr mCurrentLocation;
  IMapServer::CellPtr mTargetLocation;
  IMapServer::Path mPlannedPath;
  double mRelativeDistance;

  /*! Used for handling the behavioral finite-state-machine.
   *  In every update, it calls the current state method.
   */
  void HandleFSM();

  /*! The method setting the state of the robot behavior. It sets the function
   *  pointer for the next state, so in the next update call, the state which
   *  has been set is in execution.
   *
   *  \param pSate  a functor for the next state of robot-behavioral FSM.
   */
  void SetState(State pState);

  /*! The methods below correspond to the states of robot-behavioral FSM.
   *  Once a state is attained and after successful execution of the
   *  given state, the method sets next state or take necessary action
   *  for switching to next state.
   */

  /*! Initialization state.
   *  It is the very first state and it checks if motion state of the robot is
   *  UNINITIALIZED, only then it initiates robot homing behavior.
   */
  void StateInit();

  /*! Homing state.
   *  Homing state is used for moving the robot to reference location. This method
   *  request for motion Initialization state and starts homing timer.
   */
  void StateHoming();

  /*! Wait state for Homing.
   *  Waiting state for homing. Transit to next state on wait timeout or motion
   *  state of robot is Initialized.
   */
  void StateWaitHoming();

  /*! State for requesting job.
   *  State associated with robot for requesting the transportation job.
   */
  void StateRequestJob();

  /*! System out of service state.
   *  Corresponds to an erroneous state. This state is attained when there is
   *  an unexpected behavior or error in the system.
   */
  void StateOutOfService();

  void StateRequestPath();
  void StateWaitRequestPath();
  void StateExecutePath();
  void StateCheckPathExecution();
  void StateWaitPathExecution();
  void StateExecuteScript();

  //  void StartPickupJob(bool pStartPickupJob);
  //  bool DonePickupJob();
};
}  // namespace accmetnavigation
#endif  // DOMAIN_ROBOT_CONTROLLER_ROBOTBEHAVIORCONTROLLER_H_
