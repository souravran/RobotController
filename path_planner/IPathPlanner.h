// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef PATH_PLANNER_IPATHPLANNER_H_
#define PATH_PLANNER_IPATHPLANNER_H_

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
class IPathPlanner {
 public:
  typedef std::shared_ptr<IPathPlanner> Ptr;
  virtual ~IPathPlanner() {}

  virtual IMapServer::Path GetPath(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation) = 0;
};

}  // namespace accmetnavigation
#endif  // PATH_PLANNER_IPATHPLANNER_H_
