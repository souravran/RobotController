// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MOTION_CONTROLLER_PATHEXECUTER_H_
#define MOTION_CONTROLLER_PATHEXECUTER_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <motion_controller/IPathExecuter.h>
#include <robot_platform/IRobotPlatform.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>

namespace accmetnavigation {
/*!
 */
class PathExecuter : public IPathExecuter {
 public:
  typedef void (PathExecuter::*State)();
  typedef std::shared_ptr<PathExecuter> Ptr;
  static Ptr Create(IMapServer::Ptr pMapProxy, IRobotPlatform::Ptr pRobotPlatform);
  virtual ~PathExecuter();

  virtual void Update();
  virtual PathExecutionStates GetState();
  virtual void RequestState(PathExecutionStates pRequestedState);
  virtual std::deque<std::string>  RequestRelativePath(IMapServer::Path pPath);
  virtual bool RequestReservePath(IMapServer::Path pPath);
  virtual bool RequestUnreservePath(IMapServer::Path pPath);
  virtual void RequestDirectionChange(std::string pDirection);

 protected:
  explicit PathExecuter(IMapServer::Ptr pMapProxy, IRobotPlatform::Ptr pRobotPlatform);

 private:
  log4cpp::Category& mLogger;
  IMapServer::Ptr mMapProxyClient;
  IRobotPlatform::Ptr mRobotPlatform;
  State mExecuterState;  //!< Assigned with pointer to the motion controller FSM method
  PathExecutionStates mCurrentState;
  PathExecutionStates mRequestedState;
  std::string mRobotFaceDirection; //X,Y,mX,mY

  /*! Used for handling the motion execution finite-state-machine.
   *  In every update, it calls the current state method.
   */
  void HandleFSM();

  /*! The method for setting next state of the motion controller FSM.
   *
   *  \param pExecutionState  a functor for the next state of motion controller FSM.
   */
  void SetState(State pExecuterState);

  void StateInit();
  void StateWaitPathExecute();
  void StatePickup();
  void StateDrop();
  std::deque<std::string> ProcessPath(IMapServer::Path pPath);
};

} /* namespace accmetnavigation */
#endif  // MOTION_CONTROLLER_PATHEXECUTER_H_
