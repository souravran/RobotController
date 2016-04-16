// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef ROBOT_PLATFORM_IROBOTPLATFORM_H_
#define ROBOT_PLATFORM_IROBOTPLATFORM_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <memory>

namespace accmetnavigation {

/*! Definition of Hardware status and Connection flags.
 *  Every flag definitions holds a bit value for the 16 bit Hardware status and connection
 *  option field. Each of these flags are individually set/un-set to provide information
 *  about the ongoing status.
 */
#define CONN_FLAG 0x8000       //!< Flag for checking connection status (binary value -> 1000 0000 0000 0000)
#define HW_STATUS_FLAG 0x4000  //!< Flag for checking hardware health status (binary value -> 0100 0000 0000 0000)
#define INIT_FLAG 0x80         //!< Flag for checking robot homing status (binary value -> 0000 0000 1000 0000)
#define HALT_FLAG 0x40         //!< Flag for checking if the robot is halted (binary value -> 0000 0000 0100 0000)
#define MOTION_FLAG 0x20       //!< Flag for checking if the robot is in motion (binary value -> 0000 0000 0100 0000)

/*! An Enum class for Robot States.
 *  It defines the various motion states of the robot.
 */
enum class MotionStates {
  NONE,           //!< not a legal state
  ERROR,          //!< if there is an error in the process
  UNINITIALIZED,  //!< when the robot is not initialized
  INITIALIZING,   //!< state when the initialization process going on
  INITIALIZED,    //!< the state corresponds to robot after successfully being initialized
  READY,          //!< robot has job, planning path
  MOVING,         //!< the state when the robot is moving
  STOPPED,        //!< the sate once the robot is stopped
  HALTED          //!< the sate for emergency stopping
};

/*! A robot platform specific interface.
 *  The class encapsulates various methods that are used for handling robot behavior.
 *  Two of such platforms for the robot could be, simulation and real-world.
 */
class IRobotPlatform {
 public:
  typedef std::shared_ptr<IRobotPlatform> Ptr;
  virtual ~IRobotPlatform() {}

  /*! The method used for updating the motion states of the robot.
   *  This method gets called repeatedly in accordance with the robot-behavioral state.
   */
  virtual void Update() = 0;

  /*! The method used for getting the current motion state of the wagon.
   *  \return  The value of the current MotionState.
   */
  virtual MotionStates GetState() = 0;

  /*! The method used for requesting for a motion state of robot.
   *  Based on the requested state the application will take the necessary action to bring the robot into the
   *  requested state.
   *
   *  \param pRequestedState  one of the state {INITIALIZED, MOVING, HALTED}.
   */
  virtual void RequestState(MotionStates pRequestedState) = 0;

  /*! The method used for requesting motion of robot by setting setting relative distance and velocity values.
   *
   *  \param pRequestedRelativeDistance the requested relative distance, the robot would navigate
   *  \param pRequestedVelocity the requested velocity with which the robot would move
   */
  virtual void RequestMotion(double pRequestedRelativeDistance, double pRequestedVelocity) = 0;

  /*! This method sets a bit value of 16-bit hardware status and connection option filed.
   *
   * \param  pOptionFlag  one of the defined flag, for eg. connection flag (CONN_FLAG)
   */
  virtual void SetHWStatusConnFlags(uint16_t pOptionFlag) = 0;

  /*! This method un-sets a bit value of 16-bit hardware status and connection option filed.
   *
   * \param  pOptionFlag  one of the defined flag, for eg. connection flag (CONN_FLAG)
   */
  virtual void UnsetHWStatusConnFlags(uint16_t pOptionFlag) = 0;
};
}  // namespace accmetnavigation
#endif  // ROBOT_PLATFORM_IROBOTPLATFORM_H_
