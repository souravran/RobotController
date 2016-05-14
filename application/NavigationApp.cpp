// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <application/NavigationApp.h>
#include <robot_platform/RobotPlatformSim.h>
#include <job_scheduler/JobSimulator.h>
#include <job_scheduler/JobRequesterSimple.h>
#include <path_planner/PathPlannerAStar.h>
#include <motion_controller/PathExecuter.h>
#include <map_management/MapProxyClient.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------

log4cpp::Category& logger(log4cpp::Category::getInstance("Main"));

namespace accmetnavigation {

//! Wait time for iteratively updating the FSM
#define WAIT_IN_MILISECONDS 500

NavigationApp::NavigationApp(const int& argc, char* argv[])
: mLogger(log4cpp::Category::getInstance("NavigationApp"))
, mRobotManager() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // TODO(ssenapati): need to check for file availability and relative path

  mRobotManager = RobotManager::Create(argc, argv);
//  mMapProxyClient = MapProxyClient::Create("../MapGraph.dot");
//  mRobotPlatform = RobotPlatformSim::Create(mIOService, mSimHostAddrs, mSimDestinationPort);
//  mJobSimulator = JobSimulator::Create();
//  mJobRequester = JobRequesterSimple::Create(mJobSimulator);
//  mPathPlannerAStar = PathPlannerAStar::Create(mMapProxyClient);
//  mPathExecuter = PathExecuter::Create(mMapProxyClient, mRobotPlatform);
//  mRobotController = RobotBehaviorController::Create(mRobotPlatform, mPathExecuter, mJobRequester, mPathPlannerAStar);

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

NavigationApp::~NavigationApp() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void NavigationApp::Run() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
//  while (true) {
//    mRobotController->Update();
//    mRobotPlatform->Update();
//    mPathExecuter->Update();
//    usleep(WAIT_IN_MILISECONDS * 1000);
//  }
  mRobotManager->RunRobots();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}
}  // namespace accmetnavigation

int main(int argc, char* argv[]) {
  log4cpp::BasicConfigurator::configure();
  log4cpp::Category::getRoot().setPriority(log4cpp::Priority::DEBUG);
  logger << log4cpp::Priority::INFO << "Starting main...";

  accmetnavigation::NavigationApp naviApp(argc, argv);
  naviApp.Run();

  logger << log4cpp::Priority::INFO << "Exiting main...";
  return 0;
}
