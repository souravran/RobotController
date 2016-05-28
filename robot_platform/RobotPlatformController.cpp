// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <robot_platform/RobotPlatformController.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------

namespace accmetnavigation {

//! Wait time for iteratively checking hardware connection and health status
#define CONNECT_TIMEOUT_SECONDS 10
#define HW_STATUS_TIMEOUT_SECONDS 10


RobotPlatformController::~RobotPlatformController() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

MotionStates RobotPlatformController::GetState() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mCurrentMotionState;
}

void RobotPlatformController::RequestState(MotionStates pRequestedState) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mRequestedState = pRequestedState;
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::RequestMotion(double pRequestedRelativeDistance, double pRequestedVelocity) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mRequestedRelativeDistance = pRequestedRelativeDistance;
  mRequestedVelocity = pRequestedVelocity;
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::RequestDirection(std::string pDirection) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    mDirection = pDirection;
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::Update() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  HandleFSM();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::SetHWStatusConnFlags(uint16_t pOptionFlag) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mHWStatusConnFlags |= pOptionFlag;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::UnsetHWStatusConnFlags(uint16_t pOptionFlag) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mHWStatusConnFlags &= ~pOptionFlag;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::HandleFSM() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  (this->*mPlatformState)();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::SetState(State pPlatfomrState) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mPlatformState = pPlatfomrState;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateInit() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  SetState(&RobotPlatformController::StateConnect);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateConnect() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  if ((mHWStatusConnFlags & CONN_FLAG) > 0) {
    SetState(&RobotPlatformController::StateHWStatus);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Hardware connection failed !";
    mStartTime = boost::posix_time::microsec_clock::local_time();
    SetState(&RobotPlatformController::StateHWDisconnect);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateHWStatus() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  HWStatusCheck();
  if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
    mRequestedState = MotionStates::NONE;
    SetState(&RobotPlatformController::StateUninitialized);
  } else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Hardware connection failed !";
    SetState(&RobotPlatformController::StateConnect);
  } else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Hardware health status is not okay !";
    mStartTime = boost::posix_time::microsec_clock::local_time();
    SetState(&RobotPlatformController::StateHWError);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateUninitialized() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  HWStatusCheck();
  if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
    mCurrentMotionState = MotionStates::UNINITIALIZED;
    if (mRequestedState == MotionStates::INITIALIZED) {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Starting robot homing process...";
      SetState(&RobotPlatformController::StateStartinitialize);
    }

  } else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
    SetState(&RobotPlatformController::StateConnect);
  } else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
    SetState(&RobotPlatformController::StateHWStatus);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateStartinitialize() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  HWStatusCheck();
  // start robot homing process only when hardware is connected and hardware health status is okay
  // in addition to that, the INIT_FLAG is not set
  if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
    if ((mRequestedState == MotionStates::INITIALIZED) && (mCurrentMotionState == MotionStates::UNINITIALIZED) &&
        ((mHWStatusConnFlags & INIT_FLAG) == 0)) {
      Initialize();
      mRequestedState = MotionStates::NONE;
      SetState(&RobotPlatformController::StateWaitInitialize);
    }

  } else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
    SetState(&RobotPlatformController::StateConnect);
  } else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
    SetState(&RobotPlatformController::StateHWStatus);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateWaitInitialize() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  HWStatusCheck();
  if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
    // as we wait for robot homing, we check initialization is done and INIT_FLAG is set
    InitializationCheck();
    mCurrentMotionState = MotionStates::INITIALIZING;
    // during robot in motion, for emergency halt, we immediately stop the robot
    if (mRequestedState == MotionStates::HALTED) {
      mRequestedState = MotionStates::NONE;
      SetState(&RobotPlatformController::StateHalted);
    } else if ((mHWStatusConnFlags & INIT_FLAG) > 0) {
      mRequestedState = MotionStates::NONE;
      mRequestedRelativeDistance = 0;
      mRequestedVelocity = 0;
      mDirection.clear();
      mCurrentMotionState = MotionStates::INITIALIZED;

      SetState(&RobotPlatformController::StateStopped);
    }

  } else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
    SetState(&RobotPlatformController::StateConnect);
  } else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
    SetState(&RobotPlatformController::StateHWStatus);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateStopped() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  HWStatusCheck();
  if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
    mCurrentMotionState = MotionStates::STOPPED;
    if (mRequestedState == MotionStates::HALTED) {
      SetState(&RobotPlatformController::StateHalted);
    } else if (mRequestedState == MotionStates::MOVING) {
      SetState(&RobotPlatformController::StateStartMoving);
    }

  } else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
    SetState(&RobotPlatformController::StateConnect);
  } else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
    SetState(&RobotPlatformController::StateHWStatus);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateHalted() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  Stop();
  // After the Stop command the HALT_FLAG should have been set, then we check the flag and switch to StateStopped
  if ((mHWStatusConnFlags & HALT_FLAG) > 0) {
    mRequestedState = MotionStates::NONE;
    mRequestedRelativeDistance = 0;
    mRequestedVelocity = 0;
    SetState(&RobotPlatformController::StateStopped);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateStartMoving() {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

    ConnectionCheck();
    HWStatusCheck();
    if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
        if(!mDirection.empty()) {
            SwitchDirection(mDirection);
            mDirection.clear();
        }
        if ((mRequestedRelativeDistance != 0) && (mRequestedVelocity > 0)) {
            mRequestedState = MotionStates::NONE;
            MoveRelative(mRequestedRelativeDistance, mRequestedVelocity);
            SetState(&RobotPlatformController::StateMoving);
        }
    }
    else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
        SetState(&RobotPlatformController::StateConnect);
    }
    else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
        SetState(&RobotPlatformController::StateHWStatus);
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateMoving() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  ConnectionCheck();
  HWStatusCheck();
  if ((mHWStatusConnFlags & (CONN_FLAG | HW_STATUS_FLAG)) == (CONN_FLAG | HW_STATUS_FLAG)) {
    MotionCheck();
    if (mRequestedState == MotionStates::HALTED) {
      SetState(&RobotPlatformController::StateHalted);
    } else if ((mHWStatusConnFlags & MOTION_FLAG) > 0) {
      mCurrentMotionState = MotionStates::MOVING;
    } else if ((mHWStatusConnFlags & MOTION_FLAG) == 0) {
      mRequestedState = MotionStates::NONE;
      mRequestedRelativeDistance = 0;
      mRequestedVelocity = 0;
      SetState(&RobotPlatformController::StateStopped);
    }

  } else if ((mHWStatusConnFlags & CONN_FLAG) == 0) {
    SetState(&RobotPlatformController::StateConnect);
  } else if ((mHWStatusConnFlags & HW_STATUS_FLAG) == 0) {
    SetState(&RobotPlatformController::StateHWStatus);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateHWDisconnect() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if ((boost::posix_time::microsec_clock::local_time() - mStartTime) >
      boost::posix_time::seconds(CONNECT_TIMEOUT_SECONDS)) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Trying to connect hardware... ";
    SetState(&RobotPlatformController::StateConnect);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Waiting for re-establishing hardware connection for "
            << boost::posix_time::microsec_clock::local_time() - mStartTime << " Seconds";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotPlatformController::StateHWError() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if ((boost::posix_time::microsec_clock::local_time() - mStartTime) >
      boost::posix_time::seconds(HW_STATUS_TIMEOUT_SECONDS)) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Trying to check hardware health status... ";
    SetState(&RobotPlatformController::StateHWStatus);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Waiting to check hardware status for "
            << boost::posix_time::microsec_clock::local_time() - mStartTime << " Seconds";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

RobotPlatformController::RobotPlatformController()
: mLogger(log4cpp::Category::getInstance("RobotPlatformController"))
, mCurrentMotionState(MotionStates::NONE)
, mPlatformState(0)
, mHWStatusConnFlags(0)
, mStartTime()
, mRequestedRelativeDistance(0)
, mRequestedVelocity(0)
, mDirection() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  SetState(&RobotPlatformController::StateInit);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}
} /* namespace accmetnavigation */
