// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef ROBOT_PLATFORM_ROBOTPLATFORMSIM_H_
#define ROBOT_PLATFORM_ROBOTPLATFORMSIM_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <robot_platform/RobotPlatformController.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <sstream>
#include <istream>

namespace accmetnavigation {
/*! class for Simulation platform
 *  The class is used for handling robot operation in the simulation engine.
 *  It interacts with the simulation engine to visualize robot behaviors.
 */
class RobotPlatformSim : public RobotPlatformController {
 public:
  typedef std::shared_ptr<RobotPlatformSim> Ptr;
  static Ptr Create(boost::asio::io_service& pIOService, std::string pSimHostAddrs, int16_t pSimDestinationPort);  // NOLINT boost::asio::io_service needs non const reference
  virtual ~RobotPlatformSim();

 protected:
  RobotPlatformSim(boost::asio::io_service& pIOService, std::string pSimHostAddrs, int16_t pSimDestinationPort);  // NOLINT boost::asio::io_service needs non const reference

 private:
  log4cpp::Category& mLogger;            //!< For logging messages to the console
  boost::asio::ip::tcp::socket mSocket;  //!< Stream socket object for asynchronous communication
  std::string mSimHostAddrs;             //!< Supplied IP-address
  int16_t mSimDestinationPort;           //!< Supplied destination port number
  double mCurrentPoseVal;                //!< Holds current absolute position of robot
  double mLastPoseVal;                   //!< Holds the last absolute position of robot
  unsigned int mCountForPoseUpdate;      //!< Trivial counter, used to check if robot pose changing

  virtual void ConnectionCheck();
  virtual void HWStatusCheck();
  virtual void Initialize();
  virtual void InitializationCheck();
  virtual void Stop();
  virtual void MoveRelative(double pRelativeDistance, double pVelocity);
  virtual void MotionCheck();

  /*! The method used for writing a Telnet command for the simulation engine.
   *
   *  \param pCommand  the formulated Telnet command.
   */
  bool SendSimCommand(std::string& pCommand);  // NOLINT the Telnet command has to be passed with non-const reference

  /*! The method used for writing a Telnet command and reding the corresponding response for the simulation engine.
   *
   *  \param pCommand  the formulated Telnet command.
   *  \param pResponse  a stream buffer which is used for parsing the response of Telnet command.
   */
  bool ReceiveSimCommandResponse(std::string& pCommand, boost::asio::streambuf& pResponse);  // NOLINT boost::asio::streambuf needs non-const reference

  /*! This method parse the Telnet command response and gives out robot pose value that is of type double.
   */
  void CheckPose();
};
}  // namespace accmetnavigation
#endif  // ROBOT_PLATFORM_ROBOTPLATFORMSIM_H_
