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

IMapServer::Path PathPlannerAStar::GetPath(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

    mGraph = mMapProxyClient->GetMapGraph();
    boost::graph_traits<Map::Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
    boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
    bool found = false;

    for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
        if ((mGraph[*vertexItr]->GetProperty<unsigned int>("ID", 0) == pStartLocation->GetProperty<unsigned int>("ID", 0)) &&
           (pStartLocation->GetProperty<unsigned int>("ID", 0) != 0)) {
            found = true;
            pStartLocation = mGraph[*vertexItr];
        }
        if ((mGraph[*vertexItr]->GetProperty<unsigned int>("ID", 0) == pTargetLocation->GetProperty<unsigned int>("ID", 0)) &&
           (pTargetLocation->GetProperty<unsigned int>("ID", 0) != 0)) {
            pTargetLocation = mGraph[*vertexItr];
            found = false;
        }
        ++vertexItr;
    }

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": The start location coordinate is : ["
          << pStartLocation->GetProperty<unsigned int>("X", 0) << " , "
          << pStartLocation->GetProperty<unsigned int>("Y", 0) << "]";

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": The target location coordinate is : ["
          << pTargetLocation->GetProperty<unsigned int>("X", 0) << " , "
          << pTargetLocation->GetProperty<unsigned int>("Y", 0) << "]";

    mCameFromList.clear();
    mCostSoFarStore.clear();
    mPath.clear();
    ConductSearch(pStartLocation, pTargetLocation);
    if(mCameFromList.count(pStartLocation)) {
        mPath = ReconstructPath(pStartLocation, pTargetLocation, mCameFromList);
    }

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": The path obtained is as follows     : ";
    for (auto p : mPath) {
        mLogger << log4cpp::Priority::DEBUG << __func__ << ":   ["
              << p->GetProperty<unsigned int>("X", 0) << " , "
              << p->GetProperty<unsigned int>("Y", 0) << "]";
    }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mPath;
}

void PathPlannerAStar::ConductSearch(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    PriorityQueue<Cell::CellPtr> frontier;
    frontier.put(pStartLocation, 0);
    mCameFromList[pStartLocation] = pStartLocation;
    mCostSoFarStore[pStartLocation] = 0;
    while (!frontier.empty()) {
        auto current = frontier.get();
        if (current == pTargetLocation) {
          break;
        }
        for (auto next : mMapProxyClient->GetNeighboringCells(current)) {
            double new_cost = mCostSoFarStore[current] + 1;
            if (!mCostSoFarStore.count(next) || new_cost < mCostSoFarStore[next]) {
                mCostSoFarStore[next] = new_cost;
                int priority = new_cost + HeuristicCostEstimate(next, pTargetLocation);
                frontier.put(next, priority);
                mCameFromList[next] = current;
          }
        }
      }

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

double PathPlannerAStar::HeuristicCostEstimate(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  unsigned int x1, x2, y1, y2;
  x1 = pStartLocation->GetProperty<unsigned int>("X", 0);
  x2 = pTargetLocation->GetProperty<unsigned int>("X", 0);
  y1 = pStartLocation->GetProperty<unsigned int>("Y", 0);
  y2 = pTargetLocation->GetProperty<unsigned int>("Y", 0);
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The start Cell : [" << x1 << " , " << y1 << "]";

//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The target Cell : [" << x2 << " , " << y2 << "]";
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

IMapServer::Path PathPlannerAStar::ReconstructPath(Cell::CellPtr pStartLocation, Cell::CellPtr pTargetLocation, std::unordered_map<Cell::CellPtr, Cell::CellPtr> pCameFrom) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    //  mPath.push_back(*pTargetLocation);

    IMapServer::Path retPath;
    Cell::CellPtr currentLoc = pTargetLocation;
    retPath.push_back(currentLoc);
    while (currentLoc != pStartLocation) {
        currentLoc = mCameFromList[currentLoc];
        retPath.push_back(currentLoc);
    }
    std::reverse(retPath.begin(), retPath.end());

    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return retPath;
}

PathPlannerAStar::PathPlannerAStar(IMapServer::Ptr pMapProxy)
: mLogger(log4cpp::Category::getInstance("PathPlannerAStar"))
, mMapProxyClient(pMapProxy)
, mGraph()
, mPath()
, mStartCell(new Cell)
, mTargetCell(new Cell)
, mCameFromList()
, mCostSoFarStore() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
