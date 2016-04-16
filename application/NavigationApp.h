// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef APPLICATION_NAVIGATIONAPP_H_
#define APPLICATION_NAVIGATIONAPP_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <domain/robot_controller/RobotBehaviorController.h>
#include <robot_platform/IRobotPlatform.h>
#include <job_scheduler/IJobSimulator.h>
#include <job_scheduler/IJobRequester.h>
#include <path_planner/IPathPlanner.h>
#include <motion_controller/IPathExecuter.h>
#include <map_management/IMapServer.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/asio.hpp>
#include <list>
#include <string>

namespace accmetnavigation {
/*! The NavigationApp class.
 *  The main application class, responsible for running the system.
 *  Generates the executable for running the application.
 */
class NavigationApp {
 public:
  NavigationApp(const int &argc, char *argv[]);
  virtual ~NavigationApp();

  /*! Method for iteratively running the application.
   */
  void Run();

 private:
  log4cpp::Category &mLogger;          //!< For logging messages to the console
  boost::asio::io_service mIOService;  //!< Used as the facilitator for operating on asynchronous services
  std::string mSimHostAddrs;           //!< The host IP-address for socket connection with simulation engine
  int16_t mSimDestinationPort;         //!< The destination port for socket connection with simulation engine
  IMapServer::Ptr mMapProxyClient;
  IRobotPlatform::Ptr mRobotPlatform;  //!< Platform for handling robot operations
  IJobSimulator::Ptr mJobSimulator;
  IJobRequester::Ptr mJobRequester;
  IPathPlanner::Ptr mPathPlannerAStar;
  IPathExecuter::Ptr mPathExecuter;
  RobotBehaviorController::Ptr mRobotController;  //!< The controller for implementing behavioral-FSM
};
}  // namespace accmetnavigation
#endif  // APPLICATION_NAVIGATIONAPP_H_
