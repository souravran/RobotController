// Copyright [2015] Avantys Engineering GmbH & CoKG


//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <application/RobotManager.h>

#include <job_scheduler/JobSimulator.h>
#include <job_scheduler/JobRequesterSimple.h>


#include <map_management/MapProxyClient.h>
#include <domain/robot_controller/RobotBehaviorController.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------


namespace accmetnavigation {

//! Wait time for iteratively updating the FSM
#define WAIT_IN_MILISECONDS 50000

RobotManager::Ptr RobotManager::Create(const int &argc, char *argv[]) {
    RobotManager::Ptr retVal = RobotManager::Ptr(new RobotManager(argc, argv));
    return retVal;
}

RobotManager::~RobotManager() {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

RobotManager::RobotManager(const int &argc, char *argv[])
: mLogger(log4cpp::Category::getInstance("RobotManager"))
, mMapProxyClient()
, mJobSimulator()
, mJobRequester()
, mRobotStore(){
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    mMapProxyClient = MapProxyClient::Create("../MapGraph.dot");
    mJobSimulator = JobSimulator::Create(mMapProxyClient);
    mJobRequester = JobRequesterSimple::Create(mJobSimulator);
//    int t = 1;
    std::vector<std::string> roboNameStore;
    roboNameStore.push_back("DOXY");
//    roboNameStore.push_back("ADRIAN");

    // Robot creation and storing in container
//    while(t--) {
        std::shared_ptr<RobotUnit> newRobot1 = std::shared_ptr<RobotUnit>(new RobotUnit(mMapProxyClient, mJobRequester, roboNameStore.at(0)));
        mRobotStore.push_back(newRobot1);
//        std::shared_ptr<RobotUnit> newRobot2 = std::shared_ptr<RobotUnit>(new RobotUnit(mMapProxyClient, mJobRequester, roboNameStore.at(1)));
//        mRobotStore.push_back(newRobot2);
//    }


//    mRobotPlatform = RobotPlatformSim::Create(mIOService, mSimHostAddrs, mSimDestinationPort);
//    mPathPlannerAStar = PathPlannerAStar::Create(mMapProxyClient);
//    mPathExecuter = PathExecuter::Create(mMapProxyClient, mRobotPlatform);
//    mRobotController = RobotBehaviorController::Create(mRobotPlatform, mPathExecuter, mJobRequester, mPathPlannerAStar);
}

void RobotManager::RunRobots() {
    std::vector<std::shared_ptr<RobotUnit>>::iterator itr;

    while(true) {
        for(itr = mRobotStore.begin(); ;itr!=mRobotStore.end()) {
            itr->get()->Update();
            if((itr->get()->mRobotController->GetRobotManageState()==RobotBehaviorController::RobotManageStates::HANDLING)
               && (itr!=mRobotStore.end())) {
                itr++;
            }
            sleep(1);
        }
//        usleep(WAIT_IN_MILISECONDS * 1000000);
        sleep(1);
    }
}

} /* namespace accmetnavigation */
