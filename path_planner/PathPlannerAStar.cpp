// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <path_planner/PathPlannerAStar.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <boost/graph/astar_search.hpp>
#include <boost/make_shared.hpp>
#include <utility>
#include <functional>
#include <queue>
#include <vector>
#include <list>
#include <map>

namespace accmetnavigation {

PathPlannerAStar::Ptr PathPlannerAStar::Create(IMapServer::Ptr pMapProxy) {
  PathPlannerAStar::Ptr retVal = PathPlannerAStar::Ptr(new PathPlannerAStar(pMapProxy));
  return retVal;
}

PathPlannerAStar::~PathPlannerAStar() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

IMapServer::Path PathPlannerAStar::GetPath(IMapServer::CellPtr pStartLocation, IMapServer::CellPtr pTargetLocation) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mGraph = mMapProxyClient->GetMapGraph();
  //  IMapServer::CellPtr startLoc, targetLoc;
  boost::graph_traits<Map::Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
  boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
  bool found = false;
  for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
    if ((mGraph[*vertexItr].GetProperty<unsigned int>("ID", 0) == pStartLocation->GetProperty<unsigned int>("ID", 0)) &&
        (pStartLocation->GetProperty<unsigned int>("ID", 0) != 0)) {
      found = true;
      //      Cell s = mGraph[*vertexItr];
      IMapServer::CellPtr startLoc(new Cell(mGraph[*vertexItr]));
      pStartLocation = startLoc;
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": The start location coordinate is : ["
              << pStartLocation->GetProperty<unsigned int>("X", 0) << " , "
              << pStartLocation->GetProperty<unsigned int>("Y", 0) << "]";
    }

    if (found && (mGraph[*vertexItr].GetProperty<bool>("Navigable", false) == true)) {
      mPath.push_back(mGraph[*vertexItr]);
    }

    if ((mGraph[*vertexItr].GetProperty<unsigned int>("ID", 0) ==
         pTargetLocation->GetProperty<unsigned int>("ID", 0)) &&
        (pTargetLocation->GetProperty<unsigned int>("ID", 0) != 0)) {
      IMapServer::CellPtr targetLoc(new Cell(mGraph[*vertexItr]));
      pTargetLocation = targetLoc;

      mLogger << log4cpp::Priority::DEBUG << __func__ << ": The target location coordinate is : ["
              << pTargetLocation->GetProperty<unsigned int>("X", 0) << " , "
              << pTargetLocation->GetProperty<unsigned int>("Y", 0) << "]";
      found = false;
      //      for (auto c : mMapProxyClient->GetNeighboringCells(mGraph[*vertexItr])) {
      //        mLogger << log4cpp::Priority::DEBUG << __func__
      //                << ": The neighboring cells are :" << c.GetProperty<unsigned int>("ID", 0);
      //      }
    }

    ++vertexItr;
  }

  FScore.clear();
  openSet.clear();
  std::vector<IMapServer::CellPtr> ClosedSet;

  openSet.push_back(pStartLocation);
  std::map<IMapServer::CellPtr, IMapServer::CellPtr> CameFrom;

  std::map<unsigned int, double> GScore;
  GScore.insert(std::pair<unsigned int, double>(pStartLocation->GetProperty<unsigned int>("ID", 0), 0));

  double tentativeGScore = 0;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The first Heuristic is done "
          << pStartLocation->GetProperty<unsigned int>("ID", 0);
  FScore.insert(
      std::pair<double, IMapServer::CellPtr>(HeuristicCostEstimate(pStartLocation, pTargetLocation), pStartLocation));

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ############### 1 and Heuristic"
          << HeuristicCostEstimate(pStartLocation, pTargetLocation);

  std::vector<IMapServer::CellPtr>::const_iterator openSetItr = openSet.begin();
  //  while ((openSetItr != OpenSet.end()) && (!OpenSet.empty())) {
  while ((!openSet.empty())) {
    IMapServer::CellPtr current(new Cell);
    current = GetLowestScoreNode();
    FScore.clear();

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ------------------current node details : [ "
            << current->GetProperty<unsigned int>("X", 0) << " , " << current->GetProperty<unsigned int>("Y", 0) << " ]"
            << " And size of OpenSet : " << openSet.size() << "-----------------------";

    if (current->GetProperty<unsigned int>("ID", 0) == pTargetLocation->GetProperty<unsigned int>("ID", 0)) {
      ReconstructPath(CameFrom, current);
      return mPath;
    }

    std::vector<IMapServer::CellPtr>::size_type i = 0;
    while (i < openSet.size()) {
      if (openSet[i]->GetProperty<unsigned int>("ID", 0) == current->GetProperty<unsigned int>("ID", 0)) {
        openSet.erase(openSet.begin() + i);
      }
      i++;
    }

    ClosedSet.push_back(current);
    for (auto cell : mMapProxyClient->GetNeighboringCells(*current)) {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ": The neighbors of the current cell : [ "
              << cell.GetProperty<unsigned int>("X", 0) << " , " << cell.GetProperty<unsigned int>("Y", 0) << " ]";
      IMapServer::CellPtr neiCell(new Cell(cell));

      // if neighbor in ClosedSet
      for (auto closeCell : ClosedSet) {
        if (neiCell->GetProperty<unsigned int>("ID", 0) == closeCell->GetProperty<unsigned int>("ID", 0)) {
          break;
        }
      }
      tentativeGScore = GScore[current->GetProperty<unsigned int>("ID", 0)] + 0.0;
      // if neighbor not in OpenSet
      bool neighborPresentInOpenSet = false;
      for (auto openCell : openSet) {
        if (neiCell->GetProperty<unsigned int>("ID", 0) == openCell->GetProperty<unsigned int>("ID", 0)) {
          neighborPresentInOpenSet = true;
          mLogger << log4cpp::Priority::DEBUG << __func__ << "************ Neighbor Present in OpenSet *************";
        }
      }
      if (!neighborPresentInOpenSet) {
        openSet.push_back(neiCell);
        mLogger << log4cpp::Priority::DEBUG << __func__ << " Neighbor cell ["
                << neiCell->GetProperty<unsigned int>("X", 0) << " , " << neiCell->GetProperty<unsigned int>("Y", 0)
                << "] pushed to OpenSet";
      } else if (tentativeGScore >= GScore[neiCell->GetProperty<unsigned int>("ID", 0)]) {
        break;
      }

      CameFrom[neiCell] = current;
      GScore.insert(std::pair<unsigned int, double>(neiCell->GetProperty<unsigned int>("ID", 0), tentativeGScore));
      FScore.insert(std::pair<double, IMapServer::CellPtr>(
          (GScore[neiCell->GetProperty<unsigned int>("ID", 0)] + HeuristicCostEstimate(neiCell, pTargetLocation)),
          neiCell));
      //      FScore[GScore[neiCell->GetProperty<unsigned int>("ID", 0)] + HeuristicCostEstimate(neiCell,
      //      pTargetLocation)] =
      //          neiCell;
    }

    openSetItr++;
  }

  //  mPath.clear();
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mPath;
}

IMapServer::CellPtr PathPlannerAStar::GetLowestScoreNode() {
  std::map<double, IMapServer::CellPtr>::iterator it_min = FScore.begin();

  mLogger << log4cpp::Priority::DEBUG << __func__ << "The cells in FScore list and the costs :";
  for (auto ne : FScore) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ":  [" << ne.second->GetProperty<unsigned int>("X", 0) << " , "
            << ne.second->GetProperty<unsigned int>("Y", 0) << "  --> " << ne.first;
  }

  IMapServer::CellPtr current(new Cell);
  mLogger << log4cpp::Priority::DEBUG << __func__ << " The OpenSet cells are :";
  for (auto openCell : openSet) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ":  [" << openCell->GetProperty<unsigned int>("X", 0) << " , "
            << openCell->GetProperty<unsigned int>("Y", 0) << " ]";
    if (openCell->GetProperty<unsigned int>("ID", 0) == it_min->second->GetProperty<unsigned int>("ID", 0)) {
      current = openCell;
    }
  }
  return current;
}

template <typename Graph>
void PathPlannerAStar::a_star_search(const Graph& graph, typename Graph::Location start, typename Graph::Location goal,
                                     std::unordered_map<typename Graph::Location, typename Graph::Location>& came_from,
                                     std::unordered_map<typename Graph::Location, int>& cost_so_far) {
  typedef typename Graph::Location Location;
  PriorityQueue<Location> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    auto current = frontier.get();

    if (current == goal) {
      break;
    }

    for (auto next : graph.neighbors(current)) {
      int new_cost = cost_so_far[current] + graph.cost(current, next);
      if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        int priority = new_cost + heuristic(next, goal);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
}

double PathPlannerAStar::HeuristicCostEstimate(IMapServer::CellPtr pStartLocation,
                                               IMapServer::CellPtr pTargetLocation) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  unsigned int x1, x2, y1, y2;
  x1 = pStartLocation->GetProperty<unsigned int>("X", 0);
  x2 = pTargetLocation->GetProperty<unsigned int>("X", 0);
  y1 = pStartLocation->GetProperty<unsigned int>("Y", 0);
  y2 = pTargetLocation->GetProperty<unsigned int>("Y", 0);
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The start Cell : [" << x1 << " , " << y1 << "]";

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The target Cell : [" << x2 << " , " << y2 << "]";
  int x = x2 - x1;
  int y = y2 - y1;

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": X and Y values :" << x << " -- " << y;
  double retCost;
  retCost = pow(x, 2) + pow(y, 2);  // calculating distance by euclidean formula
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": retCost Value :" << retCost;
  retCost = sqrt(retCost);  // sqrt is function in math.h

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT " << retCost;
  return retCost;
}

void PathPlannerAStar::ReconstructPath(std::map<IMapServer::CellPtr, IMapServer::CellPtr> pCameFrom,
                                       IMapServer::CellPtr pCurrentLocation) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  //  mPath.push_back(*pTargetLocation);

  std::map<IMapServer::CellPtr, IMapServer::CellPtr>::iterator iterator;
  for (iterator = pCameFrom.begin(); iterator != pCameFrom.end(); iterator++) {
    mLogger << log4cpp::Priority::DEBUG << __func__
            << ": The path cells are : " << iterator->first->GetProperty<unsigned int>("X", 0) << "  --- "
            << iterator->second->GetProperty<unsigned int>("X", 0);
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

PathPlannerAStar::PathPlannerAStar(IMapServer::Ptr pMapProxy)
    : mLogger(log4cpp::Category::getInstance("PathPlannerAStar")),
      mMapProxyClient(pMapProxy),
      mGraph(),
      mPath(),
      mStartCell(new Cell),
      mTargetCell(new Cell),
      FScore(),
      openSet(),
	  came_from(),
	  cost_so_far() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
