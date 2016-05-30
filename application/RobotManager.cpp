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

static const std::vector<std::string> robotNameStore = {"Dala", "Roxy", "Sasa"};

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

    // Robot creation and storing in container
    for(auto robotName : robotNameStore) {
        std::shared_ptr<RobotUnit> newActiveRobot = std::shared_ptr<RobotUnit>(new RobotUnit(mMapProxyClient, mJobRequester, robotName));
        mRobotStore.push_back(newActiveRobot);
    }

//    mRobotPlatform = RobotPlatformSim::Create(mIOService, mSimHostAddrs, mSimDestinationPort);
//    mPathPlannerAStar = PathPlannerAStar::Create(mMapProxyClient);
//    mPathExecuter = PathExecuter::Create(mMapProxyClient, mRobotPlatform);
//    mRobotController = RobotBehaviorController::Create(mRobotPlatform, mPathExecuter, mJobRequester, mPathPlannerAStar);
}

void RobotManager::RunRobots() {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

    std::vector<std::shared_ptr<RobotUnit>>::iterator itrInit;
    std::vector<std::shared_ptr<RobotUnit>>::iterator itrStartInit;
    std::vector<std::shared_ptr<RobotUnit>>::iterator itrCheckInit;
    std::vector<std::shared_ptr<RobotUnit>>::iterator itrHandle;
    bool allRobotsHandling = false;
    while(true) {
        if(allRobotsHandling) {
            for(itrHandle = mRobotStore.begin(); itrHandle!=mRobotStore.end(); itrHandle++) {
                (*itrHandle)->Update();
                sleep(1);
            }
        }
        else if(!allRobotsHandling) {
            for(itrInit = mRobotStore.begin(); itrInit!=mRobotStore.end();) {
                for(itrStartInit = mRobotStore.begin(); itrStartInit<=itrInit; itrStartInit++) {
                    (*itrStartInit)->Update();
                    sleep(1);
                }
                if(((*itrInit)->mRobotController->GetRobotManageState()==RobotBehaviorController::RobotManageStates::HANDLING) &&
                        (itrInit!=mRobotStore.end())) {
                    itrInit++;
                }
            }
            for(itrCheckInit = mRobotStore.begin(); itrCheckInit!=mRobotStore.end(); itrCheckInit++) {
                if(((*itrCheckInit)->mRobotController->GetRobotManageState()==RobotBehaviorController::RobotManageStates::HANDLING)) {
                    allRobotsHandling = true;
                }
                else {
                    allRobotsHandling = false;
                }
            }
        }
//        usleep(WAIT_IN_MILISECONDS * 1000000);
//        sleep(1);
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
