// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MAP_MANAGEMENT_MAPPROXYCLIENT_H_
#define MAP_MANAGEMENT_MAPPROXYCLIENT_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/IMapServer.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <string>

namespace accmetnavigation {
/*!
 */
class MapProxyClient : public IMapServer {
 public:
  typedef std::shared_ptr<MapProxyClient> Ptr;
  static Ptr Create(std::string pGraphFilePath);
  virtual ~MapProxyClient();

  virtual Map::Graph GetMapGraph();
  virtual bool UpdateOccupancy(IMapServer::Path pUnreservedPath);
  virtual bool ReleaseOccupancy(IMapServer::Path pReservedPath);
  virtual std::list<Cell::CellPtr> GetNeighboringCells(Cell::CellPtr pLocation);
  virtual Cell::CellPtr GetCellByID(int pID);

 protected:
  explicit MapProxyClient(std::string pGraphFilePath);

 private:
  log4cpp::Category& mLogger;
  Map::Ptr mMap;
  Map::Graph mGraph;
};

} /* namespace accmetnavigation */
#endif  // MAP_MANAGEMENT_MAPPROXYCLIENT_H_
