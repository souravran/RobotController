// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef PATH_PLANNER_PATHPLANNERASTAR_H_
#define PATH_PLANNER_PATHPLANNERASTAR_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <path_planner/IPathPlanner.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <unordered_map>
#include <utility>
#include <functional>
#include <queue>
#include <vector>
#include <list>
#include <map>

namespace accmetnavigation {
/*!
 */
template <typename T, typename Number = int>
struct PriorityQueue {
  typedef std::pair<Number, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, Number priority) { elements.emplace(priority, item); }

  inline T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

// std::unordered_map<Cell::CellPtr, Cell::CellPtr> came_from;
// std::unordered_map<typename Cell, int> cost_so_far;

class PathPlannerAStar : public IPathPlanner {
 public:
  typedef std::shared_ptr<PathPlannerAStar> Ptr;

  static Ptr Create(IMapServer::Ptr pMapProxy);
  virtual ~PathPlannerAStar();

  virtual IMapServer::Path GetPath(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation);

 protected:
  explicit PathPlannerAStar(IMapServer::Ptr pMapProxy);

 private:
  log4cpp::Category& mLogger;
  IMapServer::Ptr mMapProxyClient;
  Map::Graph mGraph;
  IMapServer::Path mPath;
  Cell::CellPtr mStartCell;
  Cell::CellPtr mTargetCell;
  std::map<double, Cell::CellPtr> FScore;
  std::vector<Cell::CellPtr> openSet;

  std::unordered_map<Cell::CellPtr, Cell::CellPtr> mCameFromList;
  std::unordered_map<Cell::CellPtr, double> mCostSoFarStore;

  double HeuristicCostEstimate(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation);
  IMapServer::Path ReconstructPath(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation, std::unordered_map<Cell::CellPtr, Cell::CellPtr> pCameFrom);
  void ConductSearch(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation);
};

} /* namespace accmetnavigation */
#endif  // PATH_PLANNER_PATHPLANNERASTAR_H_
