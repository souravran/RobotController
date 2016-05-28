// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/MapProxyClient.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <memory>
#include <list>
#include <string>

namespace accmetnavigation {

MapProxyClient::Ptr MapProxyClient::Create(std::string pGraphFilePath) {
  MapProxyClient::Ptr retMapProxy = MapProxyClient::Ptr(new MapProxyClient(pGraphFilePath));
  return retMapProxy;
}

MapProxyClient::~MapProxyClient() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Map::Graph MapProxyClient::GetMapGraph() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mGraph;
}

bool MapProxyClient::UpdateOccupancy(IMapServer::Path pUnreservedPath) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  boost::graph_traits<Map::Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
  boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
  for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
    if ((mGraph[*vertexItr]->GetProperty<bool>("Navigable", false)) == true) {
      for (auto cell : pUnreservedPath) {
        if (cell->GetProperty<unsigned int>("ID", 0) == mGraph[*vertexItr]->GetProperty<unsigned int>("ID", 0)) {
          if (!(mGraph[*vertexItr]->SetProperty<bool>("Occupancy", true))) {
            return false;
          }
        }
      }
    }
    ++vertexItr;
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return true;
}

bool MapProxyClient::ReleaseOccupancy(IMapServer::Path pReservedPath) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  boost::graph_traits<Map::Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
  boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
  for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
    if ((mGraph[*vertexItr]->GetProperty<bool>("Navigable", false)) == true) {
      for (auto cell : pReservedPath) {
        if (cell->GetProperty<unsigned int>("ID", 0) == mGraph[*vertexItr]->GetProperty<unsigned int>("ID", 0)) {
          if (!(mGraph[*vertexItr]->SetProperty<bool>("Occupancy", false))) {
            return false;
          }
        }
      }
    }
    ++vertexItr;
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return true;
}

std::list<Cell::CellPtr> MapProxyClient::GetNeighboringCells(Cell::CellPtr pLocation) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

    std::list<Cell::CellPtr> retNeighbors;
    unsigned int X, X1, X2;
    unsigned int Y, Y1, Y2;
    X = pLocation->GetProperty<unsigned int>("X", 0);
    Y = pLocation->GetProperty<unsigned int>("Y", 0);
    X1 = X - 1;
    X2 = X + 1;
    Y1 = Y - 1;
    Y2 = Y + 1;
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": Neighbors for ["<<pLocation->GetProperty<unsigned int>("X", 0) <<", "<<pLocation->GetProperty<unsigned int>("Y", 0)<<"]";
    boost::graph_traits<Map::Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
    boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
    for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
        if ((mGraph[*vertexItr]->GetProperty<bool>("Navigable", false)) == true) {
            if (((mGraph[*vertexItr]->GetProperty<unsigned int>("X", 0) == X) &&
                (mGraph[*vertexItr]->GetProperty<unsigned int>("Y", 0) == Y1)) ||
                ((mGraph[*vertexItr]->GetProperty<unsigned int>("X", 0) == X) &&
                (mGraph[*vertexItr]->GetProperty<unsigned int>("Y", 0) == Y2)) ||
                ((mGraph[*vertexItr]->GetProperty<unsigned int>("X", 0) == X1) &&
                (mGraph[*vertexItr]->GetProperty<unsigned int>("Y", 0) == Y)) ||
                ((mGraph[*vertexItr]->GetProperty<unsigned int>("X", 0) == X2) &&
                (mGraph[*vertexItr]->GetProperty<unsigned int>("Y", 0) == Y))) {
                retNeighbors.push_back(mGraph[*vertexItr]);
                mLogger << log4cpp::Priority::DEBUG << __func__ << ": ["
                        << mGraph[*vertexItr]->GetProperty<unsigned int>("X", 0) << " , "
                        << mGraph[*vertexItr]->GetProperty<unsigned int>("Y", 0) << "] ";
            }
        }
        vertexItr++;
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return retNeighbors;
}

Cell::CellPtr MapProxyClient::GetCellByID(int pID) {
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    boost::graph_traits<Map::Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
    boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
    for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
        if (mGraph[*vertexItr]->GetProperty<unsigned int>("ID", 0) == pID) {
            return mGraph[*vertexItr];
        }
        vertexItr++;
    }
    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

MapProxyClient::MapProxyClient(std::string pGraphFilePath)
    : mLogger(log4cpp::Category::getInstance("MapProxyClient")), mMap(0), mGraph(0) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mMap = Map::Create(pGraphFilePath);
  mMap->CreateGraph();
  mMap->LoadGraph();
  mMap->SaveGraph();

  mGraph = mMap->GetGraph();

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
