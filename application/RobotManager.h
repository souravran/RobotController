// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef APPLICATION_ROBOTMANAGER_H_
#define APPLICATION_ROBOTMANAGER_H_

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

#include <robot_platform/RobotPlatformSim.h>
#include <path_planner/PathPlannerAStar.h>
#include <motion_controller/PathExecuter.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/asio.hpp>
#include <list>
#include <string>

namespace accmetnavigation {

class RobotManager {
public:
    typedef std::shared_ptr<RobotManager> Ptr;
    static Ptr Create(const int &argc, char *argv[]);
    virtual ~RobotManager();

    struct RobotUnit {
        boost::asio::io_service mIOService;
        std::string mSimHostAddrs;
        int16_t mSimDestinationPort;
        RobotBehaviorController::Ptr mRobotController;
        IRobotPlatform::Ptr mRobotPlatform;
        IPathPlanner::Ptr mPathPlannerAStar;
        IPathExecuter::Ptr mPathExecuter;
        IMapServer::Ptr mMapProxyClient;
        IJobRequester::Ptr mJobRequester;
        std::string mRobotName;
        RobotUnit(IMapServer::Ptr pMapProxyClient,IJobRequester::Ptr pJobRequester, std::string pRobotName)
        : mIOService()
        , mSimHostAddrs("127.0.0.1")
        , mSimDestinationPort(4000)
        , mRobotController()
        , mRobotPlatform()
        , mPathPlannerAStar()
        , mPathExecuter()
        , mMapProxyClient(pMapProxyClient)
        , mJobRequester(pJobRequester)
        , mRobotName(pRobotName) {
            mRobotPlatform = RobotPlatformSim::Create(mIOService, mSimHostAddrs, mSimDestinationPort);
            mPathPlannerAStar = PathPlannerAStar::Create(mMapProxyClient);
            mPathExecuter = PathExecuter::Create(mMapProxyClient, mRobotPlatform);
            mRobotController = RobotBehaviorController::Create(mRobotPlatform, mPathExecuter, mJobRequester, mPathPlannerAStar, mRobotName);
        }
        void Update(){
            mRobotController->Update();
            mRobotPlatform->Update();
            mPathExecuter->Update();
        }
    };
    void RunRobots();
protected:
    RobotManager(const int &argc, char *argv[]);
private:
    log4cpp::Category &mLogger;          //!< For logging messages to the console
    IMapServer::Ptr mMapProxyClient;
    IJobSimulator::Ptr mJobSimulator;
    IJobRequester::Ptr mJobRequester;
    std::vector<std::shared_ptr<RobotUnit>> mRobotStore;

};

} /* namespace accmetnavigation */

#endif /* APPLICATION_ROBOTMANAGER_H_ */
