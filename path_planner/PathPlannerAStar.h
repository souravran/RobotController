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

// std::unordered_map<IMapServer::CellPtr, IMapServer::CellPtr> came_from;
// std::unordered_map<typename Cell, int> cost_so_far;

class PathPlannerAStar : public IPathPlanner {
 public:
  typedef std::shared_ptr<PathPlannerAStar> Ptr;

  static Ptr Create(IMapServer::Ptr pMapProxy);
  virtual ~PathPlannerAStar();

  virtual IMapServer::Path GetPath(IMapServer::CellPtr pStartLocation, IMapServer::CellPtr pTargetLocation);

 protected:
  explicit PathPlannerAStar(IMapServer::Ptr pMapProxy);

 private:
  log4cpp::Category& mLogger;
  IMapServer::Ptr mMapProxyClient;
  Map::Graph mGraph;
  IMapServer::Path mPath;
  IMapServer::CellPtr mStartCell;
  IMapServer::CellPtr mTargetCell;
  std::map<double, IMapServer::CellPtr> FScore;
  std::vector<IMapServer::CellPtr> openSet;

  std::unordered_map<IMapServer::CellPtr, IMapServer::CellPtr> came_from;
  std::unordered_map<IMapServer::CellPtr, double> cost_so_far;

  double HeuristicCostEstimate(IMapServer::CellPtr pStartLocation, IMapServer::CellPtr pTargetLocation);
  void ReconstructPath(std::map<IMapServer::CellPtr, IMapServer::CellPtr> pCameFrom,
                       IMapServer::CellPtr pTargetLocation);
  IMapServer::CellPtr GetLowestScoreNode();

  template <typename Graph>
  void a_star_search(const Graph& graph, typename Graph::Location start, typename Graph::Location goal,
                     std::unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
                     std::unordered_map<typename Graph::Location, int>& cost_so_far);
};

} /* namespace accmetnavigation */
#endif  // PATH_PLANNER_PATHPLANNERASTAR_H_
