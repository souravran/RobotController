// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MAP_MANAGEMENT_IMAPSERVER_H_
#define MAP_MANAGEMENT_IMAPSERVER_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/Map.h>
#include <map_management/Cell.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <memory>
#include <list>

namespace accmetnavigation {
/*!
 */
class IMapServer {
 public:
  typedef std::shared_ptr<IMapServer> Ptr;

  typedef std::shared_ptr<Cell> CellPtr;
  typedef std::list<Cell> Path;
//  typedef Cell Cell;
  virtual ~IMapServer() {}

  virtual Map::Graph GetMapGraph() = 0;
  virtual bool UpdateOccupancy(IMapServer::Path pUnreservedPath) = 0;
  virtual std::list<Cell> GetNeighboringCells(Cell pLocation) = 0;
};
}  // namespace accmetnavigation
#endif  // MAP_MANAGEMENT_IMAPSERVER_H_
