// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <robot_platform/RobotPlatformSim.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <string>

namespace accmetnavigation {

#define CELL_LENGTH 2.2

RobotPlatformSim::Ptr RobotPlatformSim::Create(boost::asio::io_service& pIOService, std::string pSimHostAddrs,
                                               int16_t pSimDestinationPort) {
  RobotPlatformSim::Ptr retVal =
      RobotPlatformSim::Ptr(new RobotPlatformSim(pIOService, pSimHostAddrs, pSimDestinationPort));
  return retVal;
}

RobotPlatformSim::~RobotPlatformSim() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformSim::ConnectionCheck() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // for communication with simulation engine via socket, we need to keep connection alive.
  boost::system::error_code connResponse;
  try {
    if (!mSocket.is_open()) {
      boost::asio::ip::tcp::endpoint connEndpoint(boost::asio::ip::address::from_string(mSimHostAddrs),
                                                  mSimDestinationPort);
      mSocket.connect(connEndpoint, connResponse);

      if (connResponse == boost::system::errc::success) {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": Connection established : " << connResponse;
        SetHWStatusConnFlags(CONN_FLAG);
      } else {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": Could not establish connection ! ";
        throw boost::system::system_error(connResponse);
      }
    }
  } catch (std::exception& e) {
    UnsetHWStatusConnFlags(CONN_FLAG);
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Exception caught: " << e.what();
  } catch (...) {
    UnsetHWStatusConnFlags(CONN_FLAG);
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Unknown exception caught!";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformSim::HWStatusCheck() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  SetHWStatusConnFlags(HW_STATUS_FLAG);
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformSim::Initialize() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  std::stringstream commStr;
  commStr << "ID wagon.steer Initialize\r\n";
  std::string comm = commStr.str();

  if (SendSimCommand(comm)) {
    mLastPoseVal = 0;
    mCountForPoseUpdate = 0;
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot initialization command issued successfully";
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Error in robot initialization";
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return;
}

void RobotPlatformSim::InitializationCheck() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  CheckPose();
  // on Initialization, as the robot moves to the reference location,
  // the obtained wagon pose value is always non-negative and less than 1, not exactly 0.0
  if ((0 <= mCurrentPoseVal) && (mCurrentPoseVal < 1)) {
    if (mLastPoseVal == mCurrentPoseVal) {
      // when arbitrarily 3 consecutive pose values are same, meaning no pose change, then the wagon has stopped
      mCountForPoseUpdate++;
      if (mCountForPoseUpdate == 3) {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot initialized successfully";
        SetHWStatusConnFlags(INIT_FLAG);
        mLastPoseVal = 0;
        mCountForPoseUpdate = 0;
      }
    }
  }
  mLastPoseVal = mCurrentPoseVal;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformSim::Stop() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  std::stringstream commStr;
  commStr << "ID wagon.steer Initialize\r\n";
  std::string stopCommand = commStr.str();

  if (SendSimCommand(stopCommand)) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot stop command issued successfully";
    SetHWStatusConnFlags(HALT_FLAG);
    UnsetHWStatusConnFlags(MOTION_FLAG);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Error in robot initialization";
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformSim::MoveRelative(double pRelativeDistance, double pVelocity) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  std::stringstream commStr;
  commStr << "ID wagon.steer MoveRelative [" << pRelativeDistance*CELL_LENGTH << ", " << pVelocity << "]\r\n";
  std::string comm = commStr.str();

  if (SendSimCommand(comm)) {
    mLastPoseVal = mCurrentPoseVal;
    mCountForPoseUpdate = 0;
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot motion command issued successfully";
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot is moving to a distance :" << pRelativeDistance;
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Error in robot motion";
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return;
}

void RobotPlatformSim::MotionCheck() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  //  check pose change and set the motion flag and once the pose does not change unset the motion flag
  CheckPose();
  SetHWStatusConnFlags(MOTION_FLAG);

  if (mLastPoseVal == mCurrentPoseVal) {
    // when arbitrarily 3 consecutive pose values are same, meaning no pose change, then the wagon has stopped
    mCountForPoseUpdate++;
    if (mCountForPoseUpdate == 3) {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot is not in motion";
      UnsetHWStatusConnFlags(MOTION_FLAG);
      mLastPoseVal = 0;
      mCountForPoseUpdate = 0;
    }
  }

  mLastPoseVal = mCurrentPoseVal;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

bool RobotPlatformSim::SendSimCommand(std::string& pCommand) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  bool retVal = false;
  try {
    boost::system::error_code errSendMessage;
    boost::asio::write(mSocket, boost::asio::buffer(pCommand), boost::asio::transfer_all(), errSendMessage);
    retVal = true;
  } catch (std::exception& e) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Exception caught: " << e.what();
  } catch (...) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Unknown exception caught!";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return retVal;
}

bool RobotPlatformSim::ReceiveSimCommandResponse(std::string& pCommand, boost::asio::streambuf& pResponse) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  bool retVal = false;
  try {
    boost::system::error_code errReceiveMessage;
    boost::asio::write(mSocket, boost::asio::buffer(pCommand), boost::asio::transfer_all(), errReceiveMessage);
    // read response from the socket stream
    boost::asio::read_until(mSocket, pResponse, " ]");
    retVal = true;
  } catch (std::exception& e) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Exception caught: " << e.what();
  } catch (...) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Unknown exception caught!";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return retVal;
}

void RobotPlatformSim::CheckPose() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  boost::asio::streambuf response;

  std::stringstream commStr;
  commStr << "ID wagon.steer TrackPose\r\n";
  std::string comm = commStr.str();
  if (ReceiveSimCommandResponse(comm, response)) {
    std::string ID, commandRet, startDelim, poseValue, endDelim;
    std::istream poseStream(&response);
    poseStream >> ID >> commandRet >> startDelim >> poseValue >> endDelim;
    // the TrackPose service returns SUCCESS, on the first Telnet-command is fired
    if (poseValue.compare("SUCCESS") != 0) {
      mCurrentPoseVal = std::stod(poseValue);
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": The Pose received : " << mCurrentPoseVal;
    }
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

RobotPlatformSim::RobotPlatformSim(boost::asio::io_service& pIOService, std::string pHostAddrs,
                                   int16_t pDestinationPort)
    : mLogger(log4cpp::Category::getInstance("RobotPlatformSim")),
      mSocket(pIOService),
      mSimHostAddrs(pHostAddrs),
      mSimDestinationPort(pDestinationPort),
      mCurrentPoseVal(0),
      mLastPoseVal(0),
      mCountForPoseUpdate(0) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

}  // namespace accmetnavigation
