// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <motion_controller/PathExecuter.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------

namespace accmetnavigation {

PathExecuter::Ptr PathExecuter::Create(IMapServer::Ptr pMapProxy, IRobotPlatform::Ptr pRobotPlatform) {
  PathExecuter::Ptr retVal = PathExecuter::Ptr(new PathExecuter(pMapProxy, pRobotPlatform));
  return retVal;
}

PathExecuter::~PathExecuter() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::Update() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  HandleFSM();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

PathExecutionStates PathExecuter::GetState() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mCurrentState;
}

void PathExecuter::RequestState(PathExecutionStates pRequestedState) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mRequestedState = pRequestedState;
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

double PathExecuter::RequestRelativePath(IMapServer::Path pPath) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  // the cells are considered to be of unit length
  // performing the cell-length calibration in robot_platform

  // as currently the cells are considered to be unit length,
  // the relative distance is nothing but the number of cells in the path
  double relativeDistance = pPath.size();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return relativeDistance;
}

bool PathExecuter::RequestReservePath(IMapServer::Path pUnreservedPath) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  bool retSuccess = mMapProxyClient->UpdateOccupancy(pUnreservedPath);
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return retSuccess;
}

void PathExecuter::HandleFSM() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  (this->*mExecuterState)();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::SetState(State pExecuterState) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mExecuterState = pExecuterState;
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::StateInit() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if (mRobotPlatform->GetState() == MotionStates::STOPPED) {
    SetState(&PathExecuter::StateWaitPathExecute);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::StateWaitPathExecute() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if (mRequestedState == PathExecutionStates::EXECUTE_PATH) {
    if ((mCurrentState == PathExecutionStates::NONE) || (mCurrentState == PathExecutionStates::DROP_REACHED)) {
      SetState(&PathExecuter::StatePickup);
    } else if (mCurrentState == PathExecutionStates::PICKUP_REACHED) {
      SetState(&PathExecuter::StateDrop);
    }
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::StatePickup() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if (mRobotPlatform->GetState() == MotionStates::MOVING) {
    mCurrentState = PathExecutionStates::DROP;
  } else if (mRobotPlatform->GetState() == MotionStates::STOPPED) {
    // also need to check if the wagon is at target cell, currently just checking if robot has stopped after moving
    mCurrentState = PathExecutionStates::PICKUP_REACHED;
    mRequestedState = PathExecutionStates::NONE;
    SetState(&PathExecuter::StateWaitPathExecute);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::StateDrop() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if (mRobotPlatform->GetState() == MotionStates::MOVING) {
    mCurrentState = PathExecutionStates::DROP;
  } else if (mRobotPlatform->GetState() == MotionStates::STOPPED) {
    // also need to check if the wagon is at target cell, currently just checking if robot has stopped after moving
    mCurrentState = PathExecutionStates::DROP_REACHED;
    mRequestedState = PathExecutionStates::NONE;
    SetState(&PathExecuter::StateWaitPathExecute);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

PathExecuter::PathExecuter(IMapServer::Ptr pMapProxy, IRobotPlatform::Ptr pRobotPlatform)
    : mLogger(log4cpp::Category::getInstance("MotionExecuter")),
      mMapProxyClient(pMapProxy),
      mRobotPlatform(pRobotPlatform),
      mExecuterState(0),
      mCurrentState(PathExecutionStates::NONE),
      mRequestedState(PathExecutionStates::NONE) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  SetState(&PathExecuter::StateInit);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}
} /* namespace accmetnavigation */
