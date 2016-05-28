// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef ROBOT_PLATFORM_ROBOTPLATFORMCONTROLLER_H_
#define ROBOT_PLATFORM_ROBOTPLATFORMCONTROLLER_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <robot_platform/IRobotPlatform.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace accmetnavigation {
/*! class for motion FSM
 *  The class abstracts away the platform specific details and implements
 *  the motion FSM.
 */
class RobotPlatformController : public IRobotPlatform {
 public:
  typedef void (RobotPlatformController::*State)();
  virtual ~RobotPlatformController();

  virtual MotionStates GetState();
  virtual void RequestState(MotionStates pRequestedState);
  virtual void RequestMotion(double pRequestedRelativeDistance, double pRequestedVelocity);
  virtual void RequestDirection(std::string pDirection);
  virtual void Update();

 protected:
  RobotPlatformController();
  virtual void SetHWStatusConnFlags(uint16_t pOptionFlag);
  virtual void UnsetHWStatusConnFlags(uint16_t pOptionFlag);

  /*! The method used for checking the hardware connection status.
   */
  virtual void ConnectionCheck() = 0;

  /*! The method used for checking the hardware health status.
   */
  virtual void HWStatusCheck() = 0;

  /*! The method used for executing robot homing process.
   */
  virtual void Initialize() = 0;

  /*! Method for checking if robot initialization done.
   */
  virtual void InitializationCheck() = 0;

  /*! Method for stopping the robot.
   */
  virtual void Stop() = 0;

  /*! Method for moving the robot.
   *
   * \param pRelativeDistance the supplied relative distance, the robot has to move
   * \param pVelocity the velocity with which the robot required to move
   */
  virtual void MoveRelative(double pRelativeDistance, double pVelocity) = 0;

  virtual void SwitchDirection(std::string pDirection) = 0;

  /*! Method for checking whether robot is in motion.
   */
  virtual void MotionCheck() = 0;

 private:
  log4cpp::Category& mLogger;           //!< For logging messages to the console
  MotionStates mRequestedState;         //!< Holds the requested motion state
  MotionStates mCurrentMotionState;     //!< Holds the current motion state
  State mPlatformState;                 //!< Assigned with pointer to the robot-platform FSM method
  uint16_t mHWStatusConnFlags;          //!< Holds the 16-bit option field for Hardware status and Connection flags
  boost::posix_time::ptime mStartTime;  //!< Used to track time for any waiting-state operation
  double mRequestedRelativeDistance;
  double mRequestedVelocity;
  std::string mDirection;

  /*! Used for handling the motion finite-state-machine.
   *  In every update, it calls the current state method.
   */
  void HandleFSM();

  /*! The method for setting next state of the robot-platform FSM. It sets the
   *  function pointer for the next state, so in the next update call, the state
   *  which has been set is in execution.
   *
   *  \param pPlatformState  a functor for the next state of robot-platform FSM.
   */
  void SetState(State pPlatformState);

  /*! The methods below correspond to the states of robot-platform FSM.
   *  Once a state is attained and after successful execution of the
   *  given state, the method sets next state or take necessary action
   *  for switching to next state. The state change is in accordance
   *  with the robot-behavioral states.
   */
  void StateInit();
  void StateConnect();
  void StateHWStatus();
  void StateUninitialized();
  void StateStartinitialize();
  void StateWaitInitialize();
  void StateStopped();
  void StateHalted();
  void StateStartMoving();
  void StateMoving();
  void StateHWDisconnect();
  void StateHWError();
};
} /* namespace accmetnavigation */
#endif  // ROBOT_PLATFORM_ROBOTPLATFORMCONTROLLER_H_
