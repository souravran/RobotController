// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MOTION_CONTROLLER_IPATHEXECUTER_H_
#define MOTION_CONTROLLER_IPATHEXECUTER_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/IMapServer.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <memory>

namespace accmetnavigation {
/*!
 */
enum class PathExecutionStates {
  NONE,   //!< not a legal state
  ERROR,  //!< if there is an error in the process
  EXECUTE_PATH,
  REACHED,
  PICKUP,
  PICKUP_REACHED,
  DROP,
  DROP_REACHED,
};

class IPathExecuter {
 public:
  typedef std::shared_ptr<IPathExecuter> Ptr;
  virtual ~IPathExecuter() {}

  virtual void Update() = 0;
  virtual PathExecutionStates GetState() = 0;
  virtual void RequestState(PathExecutionStates pRequestedState) = 0;
  virtual std::deque<std::string> RequestRelativePath(IMapServer::Path pPath) = 0;
  virtual bool RequestReservePath(IMapServer::Path pUnreservedPath) = 0;
  virtual bool RequestUnreservePath(IMapServer::Path pUnreservedPath) = 0;
  virtual void RequestDirectionChange(std::string pDirection) = 0;
  virtual void RequestReleaseCell(IMapServer::Path pPlannedPath) = 0;
};
}  // namespace accmetnavigation
#endif  // MOTION_CONTROLLER_IPATHEXECUTER_H_
