// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MAP_MANAGEMENT_MAP_H_
#define MAP_MANAGEMENT_MAP_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/Cell.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <memory>
#include <vector>
#include <string>

namespace accmetnavigation {
/*!
 */
class Map {
 public:
  typedef std::shared_ptr<Map> Ptr;

  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, Cell::CellPtr> Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  static Ptr Create(std::string pGraphFilePath);
  virtual ~Map();

  void CreateGraph();
  void LoadGraph();
  void SaveGraph();
  Map::Graph GetGraph();

 protected:
  explicit Map(std::string pGraphFilePath);

 private:
  log4cpp::Category& mLogger;
  Graph mGraph;
  std::shared_ptr<Vertex> mVertex;
  boost::dynamic_properties mPropertiesRead;
  boost::dynamic_properties mPropertiesWrite;
  // this is probably useless since the graph stores everything,
  // haven't looked for a way around it yet
  std::vector<std::vector<Vertex>> mGrid;
  std::string mGraphFilePath;
};

} /* namespace accmetnavigation */
#endif  // MAP_MANAGEMENT_MAP_H_
