// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <domain/robot_controller/RobotBehaviorController.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <list>

namespace accmetnavigation {

//! Time interval for Homing process
#define HOMING_TIMEOUT_SECONDS 40
#define PATHPLANNING_TIMEOUT_SECONDS 10

RobotBehaviorController::Ptr RobotBehaviorController::Create(IRobotPlatform::Ptr pRobotPlatform,
                                                             IPathExecuter::Ptr pMotionExecuter,
                                                             IJobRequester::Ptr pJobRequester,
                                                             IPathPlanner::Ptr pPathPlanner) {
  RobotBehaviorController::Ptr retVal = RobotBehaviorController::Ptr(
      new RobotBehaviorController(pRobotPlatform, pMotionExecuter, pJobRequester, pPathPlanner));
  return retVal;
}

RobotBehaviorController::~RobotBehaviorController() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::Update() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  HandleFSM();

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::HandleFSM() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  (this->*mState)();

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::SetState(State pState) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mState = pState;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateInit() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if (mRobotPlatform->GetState() == MotionStates::UNINITIALIZED) {
    SetState(&RobotBehaviorController::StateHoming);
  } else if (mRobotPlatform->GetState() == MotionStates::INITIALIZING) {
    SetState(&RobotBehaviorController::StateWaitHoming);
  } else if (mRobotPlatform->GetState() == MotionStates::INITIALIZED) {
    SetState(&RobotBehaviorController::StateRequestJob);
  } else if (mRobotPlatform->GetState() == MotionStates::ERROR) {
    SetState(&RobotBehaviorController::StateOutOfService);
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateHoming() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mRobotPlatform->RequestState(MotionStates::INITIALIZED);
  mStartTime = boost::posix_time::microsec_clock::local_time();
  SetState(&RobotBehaviorController::StateWaitHoming);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateWaitHoming() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if ((boost::posix_time::microsec_clock::local_time() - mStartTime) >
      boost::posix_time::seconds(HOMING_TIMEOUT_SECONDS)) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Homing failed due to time out! ";

    // time out on homing, set the next state to Out-Of-Service
    SetState(&RobotBehaviorController::StateOutOfService);
  } else {
    if (mRobotPlatform->GetState() == MotionStates::INITIALIZED) {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Homing done in "
              << boost::posix_time::microsec_clock::local_time() - mStartTime << " Seconds";

      // we need the current robot location (Cell) for requesting Job and planing path till the pick-up location
      // also to set the current location with cell from where the rail-track begins,
      // e.i. start location after homing done

      // both target and current location are arbitrarily the same, once the robot is initialized
      mCurrentLocation->SetProperty<unsigned int>("ID", 14);
      mTargetLocation = mCurrentLocation;
      SetState(&RobotBehaviorController::StateRequestJob);
    } else {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Homing is going on for "
              << boost::posix_time::microsec_clock::local_time() - mStartTime << " Seconds";
    }
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateRequestJob() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mCurrentJob = mJobRequesterSimple->RequestJob();
  if (mCurrentJob) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Obtained Job, with ID : " << mCurrentJob->GetID();
    SetState(&RobotBehaviorController::StateRequestPath);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": No transportation job available";
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateOutOfService() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateRequestPath() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mCurrentLocation = mTargetLocation;
  if (mPathExecuter->GetState() == PathExecutionStates::PICKUP_REACHED) {
    mTargetLocation = mCurrentJob->GetDropLocation();
  } else {
    mTargetLocation = mCurrentJob->GetPickupLocation();
  }

  mStartTime = boost::posix_time::microsec_clock::local_time();
  mPlannedPath = mPathPlanner->GetPath(mCurrentLocation, mTargetLocation);

  mRelativeDistance = 0;
  SetState(&RobotBehaviorController::StateWaitRequestPath);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateWaitRequestPath() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if ((boost::posix_time::microsec_clock::local_time() - mStartTime) >
      boost::posix_time::seconds(PATHPLANNING_TIMEOUT_SECONDS)) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Path planning failed due to time out! ";

    // time out on path planning, set the next state to Out-Of-Service
    SetState(&RobotBehaviorController::StateOutOfService);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__
            << ": Planning path for :" << boost::posix_time::microsec_clock::local_time() - mStartTime << " Seconds";
    if (!mPlannedPath.empty()) {
      if (mPathExecuter->RequestReservePath(mPlannedPath)) {
        if (mRobotPlatform->GetState() == MotionStates::STOPPED) {
          mRelativeDistance = mPathExecuter->RequestRelativePath(mPlannedPath);
          SetState(&RobotBehaviorController::StateExecutePath);
        }
      } else {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": Path could not be reserved for now ";
      }
    } else {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Not a valid path";
      mPlannedPath = mPathPlanner->GetPath(mCurrentLocation, mTargetLocation);
    }
    // check if valid path obtained
    // request for path reserving

    // if there is a valid path and it has been reserved, then request for relative distance
    // currently doing nothing here other than printing a log message,
    // because this is a wait state for path planning and path reserving
    // lets assume that path planning and reserving has been done in 3 seconds
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateExecutePath() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if ((mRobotPlatform->GetState() == MotionStates::STOPPED) && (mRelativeDistance != 0)) {
    mRobotPlatform->RequestState(MotionStates::MOVING);
    // just setting dummy relative distance and velocity values for the current test job
    mRobotPlatform->RequestMotion(mRelativeDistance, 20.0);
    SetState(&RobotBehaviorController::StateCheckPathExecution);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateCheckPathExecution() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if (mRobotPlatform->GetState() == MotionStates::MOVING) {
    mPathExecuter->RequestState(PathExecutionStates::EXECUTE_PATH);
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot has started moving";
    SetState(&RobotBehaviorController::StateWaitPathExecution);
  } else {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Checking if robot is moving";
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateWaitPathExecution() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if (mRobotPlatform->GetState() == MotionStates::MOVING) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot moving to the target ...";
  } else if (mRobotPlatform->GetState() == MotionStates::STOPPED) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot has stopped moving";

    // when the robot is stopped, check if it is in target location
    // once the robot at target location, then we need to execute the script
    SetState(&RobotBehaviorController::StateExecuteScript);

  } else if (mRobotPlatform->GetState() == MotionStates::ERROR) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << " Error in robot path execution !!";
    SetState(&RobotBehaviorController::StateOutOfService);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void RobotBehaviorController::StateExecuteScript() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // if job is done, then request a job, otherwise again request path to drop location
  if (mPathExecuter->GetState() == PathExecutionStates::PICKUP_REACHED) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot is picking up the sample";
    SetState(&RobotBehaviorController::StateRequestPath);
  } else if (mPathExecuter->GetState() == PathExecutionStates::DROP_REACHED) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Robot is dropping the sample";
    mCurrentJob = 0;
    SetState(&RobotBehaviorController::StateRequestJob);
  }

  // the robot is at target location, hence a new path has be planned
  mPlannedPath.clear();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

RobotBehaviorController::RobotBehaviorController(IRobotPlatform::Ptr pRobotPlatform, IPathExecuter::Ptr pMotionExecuter,
                                                 IJobRequester::Ptr pJobRequester, IPathPlanner::Ptr pPathPlanner)
    : mLogger(log4cpp::Category::getInstance("RobotBehaviorController")),
      mRobotPlatform(pRobotPlatform),
      mPathExecuter(pMotionExecuter),
      mJobRequesterSimple(pJobRequester),
      mPathPlanner(pPathPlanner),
      mCurrentJob(0),
      mState(0),
      mStartTime(),
      mCurrentLocation(new Cell),
      mTargetLocation(new Cell),
      mPlannedPath(),
      mRelativeDistance(0) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  SetState(&RobotBehaviorController::StateInit);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}
}  // namespace accmetnavigation
