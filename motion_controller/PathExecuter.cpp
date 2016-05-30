// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <motion_controller/PathExecuter.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------

namespace accmetnavigation {

static const std::map<int, int> JunctionCells = {{125, 1}, {118, 1}, {269, 1}, {262, 1}, {254, 1}, {406, 1}};

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

std::deque<std::string>  PathExecuter::RequestRelativePath(IMapServer::Path pPath) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  // the cells are considered to be of unit length
  // performing the cell-length calibration in robot_platform
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The Path coordinates are    :";
  for(auto c : pPath) {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": ["
              << c->GetProperty<unsigned int>("X", 0) << " , "
              << c->GetProperty<unsigned int>("Y", 0) << "] ";
  }

  // as currently the cells are considered to be unit length,
  // the relative distance is nothing but the number of cells in the path
  mRelativeDist = abs(pPath.size());
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return ProcessPath(pPath);
}

bool PathExecuter::RequestReservePath(IMapServer::Path pUnreservedPath) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    bool retSuccess = mMapProxyClient->UpdateOccupancy(pUnreservedPath);
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return retSuccess;
}

bool PathExecuter::RequestUnreservePath(IMapServer::Path pReservedPath) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    // pop back the last cell, cause currently the robot stays there
//    pReservedPath.pop_back();
    bool retSuccess = mMapProxyClient->ReleaseOccupancy(pReservedPath);
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return retSuccess;
}

void PathExecuter::RequestDirectionChange(std::string pDirection) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    if(mRobotFaceDirection == "X") {
        if(pDirection == "l") {
            mRobotFaceDirection = "mY";
        }
        else if(pDirection == "r") {
            mRobotFaceDirection = "Y";
        }
    }
    else if(mRobotFaceDirection == "mX") {
        if(pDirection == "l") {
            mRobotFaceDirection = "Y";
        }
        else if(pDirection == "r") {
            mRobotFaceDirection = "mY";
        }
    }
    else if(mRobotFaceDirection == "Y") {
        if(pDirection == "l") {
            mRobotFaceDirection = "X";
        }
        else if(pDirection == "r") {
            mRobotFaceDirection = "mX";
        }
    }
    else {
        if(pDirection == "l") {
            mRobotFaceDirection = "mX";
        }
        else if(pDirection == "r") {
            mRobotFaceDirection = "X";
        }
    }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::RequestReleaseCell(IMapServer::Path pPlannedPath) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    int numCells = (int)(mRelativeDist - mRobotPlatform->GetPose());
    if(pPlannedPath.size()>numCells) {
       for(int i=0; i<numCells; i++) {
           pPlannedPath.pop_front();
           mMapProxyClient->ReleaseOccupancy(pPlannedPath);
       }
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
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
        mRequestedState = PathExecutionStates::NONE;
        if ((mCurrentState == PathExecutionStates::NONE) || (mCurrentState == PathExecutionStates::DROP_REACHED)) {
            SetState(&PathExecuter::StatePickup);
        }
        else if (mCurrentState == PathExecutionStates::PICKUP_REACHED) {
            SetState(&PathExecuter::StateDrop);
        }
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void PathExecuter::StatePickup() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  if (mRobotPlatform->GetState() == MotionStates::MOVING) {
    mCurrentState = PathExecutionStates::PICKUP;
  } else if ((mRobotPlatform->GetState() == MotionStates::STOPPED) && (mRequestedState == PathExecutionStates::REACHED)) {
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
  } else if ((mRobotPlatform->GetState() == MotionStates::STOPPED) && (mRequestedState == PathExecutionStates::REACHED)) {
    // also need to check if the wagon is at target cell, currently just checking if robot has stopped after moving
    mCurrentState = PathExecutionStates::DROP_REACHED;
    mRequestedState = PathExecutionStates::NONE;
    SetState(&PathExecuter::StateWaitPathExecute);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

std::deque<std::string>  PathExecuter::ProcessPath(IMapServer::Path pPath) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    // lets check the first two elements of path to decide motion direction
    int x1,x2,y1,y2;
    std::string direction;
    std::string motionDir;
    std::deque<std::string> cmd;
    IMapServer::Path::iterator itr = pPath.begin();
    x1 = (*itr)->GetProperty<unsigned int>("X", 0);
    y1 = (*itr)->GetProperty<unsigned int>("Y", 0);
    itr++;
    x2 = (*itr)->GetProperty<unsigned int>("X", 0);
    y2 = (*itr)->GetProperty<unsigned int>("Y", 0);
    if(x1 == x2) {
        // aligned to Y-axis
        if(y2 > y1) { // towards positive y
            direction = "Y";
        }
        else {
            direction = "mY";
        }
        if(direction == mRobotFaceDirection) {
            motionDir = "";
        }
        else {
            motionDir = "-";
        }
    }
    else if(y1 == y2) {
        // aligned to X-axis
        if(x2 > x1) { // towards positive x
            direction = "X";
        }
        else {
            direction = "mX";
        }
        if(direction == mRobotFaceDirection) {
            motionDir = "";
        }
        else {
            motionDir = "-";
        }
    }
    direction = mRobotFaceDirection;
    itr--;
    x1 = (*itr)->GetProperty<unsigned int>("X", 0);
    y1 = (*itr)->GetProperty<unsigned int>("Y", 0);
    itr++;
    bool hasXStraightPath = false;
    bool hasYStraightPath = false;

    for(;itr != pPath.end();itr++) {
        x2 = (*itr)->GetProperty<unsigned int>("X", 0);
        y2 = (*itr)->GetProperty<unsigned int>("Y", 0);
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": "<< "  ID :"<<(*itr)->GetProperty<unsigned int>("ID",0);
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": ["<<x1<<","<<y1<<"]     ["<<x2<<","<<y2<<"]    direction :"<<direction;

        if((y1==y2) && (x1!=x2)) {
            hasYStraightPath = false;
            if((direction == "X") || (direction == "mX")) {

                if(hasXStraightPath) {
                    cmd.pop_back();
                    cmd.push_back("s");
                    cmd.push_back(motionDir+"1.5");
                    cmd.push_back("s");
                    hasXStraightPath = false;
                }
                cmd.push_back(motionDir+"1");
                if(x2>x1) {
                    direction = "X";
                }
                else {
                    direction = "mX";
                }
                if(JunctionCells.count((*itr)->GetProperty<unsigned int>("ID",0)) == 1) {
                    hasXStraightPath = true;
                }
            }
            else {
                 cmd.pop_back();
                 if(direction == "Y") {
                     if(x2>x1) {
                         cmd.push_back("l");
                         direction = "X";
                     }
                     else {
                         cmd.push_back("r");
                         direction = "mX";
                     }
                 }
                 else {
                     if(x2>x1) {
                         cmd.push_back("r");
                         direction = "X";
                     }
                     else {
                         cmd.push_back("l");
                         direction = "mX";
                     }
                 }
                 cmd.push_back(motionDir+"1.5");
                 cmd.push_back("s");
            }
        }
        if((x1==x2) && (y1!=y2)) {
            hasXStraightPath = false;
            if((direction == "Y") || (direction == "mY")) {
                if(hasYStraightPath) {
                    cmd.pop_back();
                    cmd.push_back("s");
                    cmd.push_back(motionDir+"1.5");
                    cmd.push_back("s");
                    hasYStraightPath = false;
                }
                cmd.push_back(motionDir+"1");
                if(y2>y1) {
                    direction = "Y";
                }
                else {
                    direction = "mY";
                }
                if(JunctionCells.count((*itr)->GetProperty<unsigned int>("ID",0)) == 1) {
                    hasYStraightPath = true;
                }
            }
            else {
                 cmd.pop_back();
                 if(direction == "X") {
                     if(y2>y1) {
                         cmd.push_back("r");
                         direction = "Y";
                     }
                     else {
                         cmd.push_back("l");
                         direction = "mY";
                     }
                 }
                 else {
                     if(y2>y1) {
                         cmd.push_back("l");
                         direction = "Y";
                     }
                     else {
                         cmd.push_back("r");
                         direction = "mY";
                     }
                 }
                 cmd.push_back(motionDir+"1.5");
                 cmd.push_back("s");
            }
        }
        x1 = x2;
        y1 = y2;
    }
    for(auto n : cmd) {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": "<< n;
    }

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": The path command is as follows :";
    double unit = 0.0;
    std::deque<std::string> newCmd;
    for (auto c : cmd) {
        if((c == "l") || (c == "r") || (c == "s") || (c == "1.5") || (c == "-1.5")) {
            if(unit) {
                newCmd.push_back(std::to_string(unit));
                unit = 0.0;
            }
            newCmd.push_back(c);
        }
        else {
            if(c == "-1") {
                unit--;
            }
            else {
                unit++;
            }
        }
    }
    if(unit) {
    	newCmd.push_back(std::to_string(unit));
    }
    for(auto n : newCmd) {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ": "<< n;
    }

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return newCmd;
}

PathExecuter::PathExecuter(IMapServer::Ptr pMapProxy, IRobotPlatform::Ptr pRobotPlatform)
: mLogger(log4cpp::Category::getInstance("PathExecuter"))
, mMapProxyClient(pMapProxy)
, mRobotPlatform(pRobotPlatform)
, mExecuterState(0)
, mCurrentState(PathExecutionStates::NONE)
, mRequestedState(PathExecutionStates::NONE)
, mRobotFaceDirection("X")
, mRelativeDist(0) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  SetState(&PathExecuter::StateInit);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}
} /* namespace accmetnavigation */
