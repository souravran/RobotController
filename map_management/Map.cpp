// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/Map.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <iostream>
#include <string>
#include <fstream>

namespace accmetnavigation {

// current mapped graph from the simulation model is 16X16 grid
const unsigned int RowNum = 16;
const unsigned int ColumnNum = 16;

// the navigable cells are identified from the simulation model grid,
// and supplied for statically building the graph object
static const int NavigableCells[] = {14,  30,  46,  62,  78,  94,  110, 126, 142, 141, 140, 139, 138, 137,
                                     136, 135, 134, 133, 132, 131, 130, 146, 162, 178, 194, 210, 226, 242};

Map::Ptr Map::Create(std::string pGraphFilePath) {
  Map::Ptr retMap = Map::Ptr(new Map(pGraphFilePath));
  return retMap;
}

Map::~Map() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}
void Map::CreateGraph() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // creating the graph structure and setting all Cell properties to the default values
  int dummyID = 1;
  for (unsigned int row = 0; row < RowNum; row++) {
    mGrid.resize(RowNum);
    for (unsigned int col = 0; col < ColumnNum; col++) {
      mGrid[row].resize(ColumnNum);
//      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Creating Map " << dummyID;
      // by default all the cells are NOT navigable but that won't always be the case
      mVertex = std::shared_ptr<Vertex>(new Vertex(boost::add_vertex(mGraph)));
      if(*mVertex == 0) {
          mLogger << log4cpp::Priority::DEBUG << __func__ << ": Non-Empty Vertex";
      }
      Cell::CellPtr cs = Cell::CellPtr(new Cell());
      cs->SetProperty<unsigned int>("ID", dummyID);
      cs->SetProperty<bool>("Navigable", false);
      cs->SetProperty<unsigned int>("X", row);
      cs->SetProperty<unsigned int>("Y", col);
      cs->SetProperty<bool>("Occupancy", false);

      mGraph[*mVertex] = cs;

//      mGraph[*mVertex]->SetProperty<unsigned int>("ID", dummyID);
//      mGraph[*mVertex]->SetProperty<bool>("Navigable", false);
//      mGraph[*mVertex]->SetProperty<unsigned int>("X", row);
//      mGraph[*mVertex]->SetProperty<unsigned int>("Y", col);
//      mGraph[*mVertex]->SetProperty<bool>("Occupancy", false);
      mGrid[row][col] = *mVertex;
      dummyID++;

//      mLogger << log4cpp::Priority::DEBUG << __func__ << ": Done creating Map " << dummyID;

      // add the edges for the contained cells in the grid
      if (col > 0) {
        boost::add_edge(mGrid[row][col - 1], mGrid[row][col], mGraph);
      }
      if (row > 0) {
        boost::add_edge(mGrid[row - 1][col], mGrid[row][col], mGraph);
      }
    }
  }

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void Map::LoadGraph() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // the default graph has been created, now setting the Cell properties of the graph with actual values statically
  // currently the only relevant Cell property is "Navigable" property,
  // setting the Navigable property of the graph object with the supplied cell IDs

  boost::graph_traits<Graph>::vertex_iterator vertexStart, vertexEnd, vertexItr;
  boost::tie(vertexStart, vertexEnd) = boost::vertices(mGraph);
  for (vertexItr = vertexStart; vertexStart != vertexEnd; vertexStart = vertexItr) {
    if ((std::end(NavigableCells) != std::find(std::begin(NavigableCells), std::end(NavigableCells),
                                               mGraph[*vertexItr]->GetProperty<unsigned int>("ID", 0)))) {
      mGraph[*vertexItr]->SetProperty<bool>("Navigable", true);
    }
    ++vertexItr;
  }
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": The navigable property of cells has been set";

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

void Map::SaveGraph() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  // write cell properties into the DOT file
//  auto valueID = boost::make_transform_value_property_map(
//      [](Cell &cell) { return cell.GetProperty<unsigned int>("ID", 0); }, boost::get(boost::vertex_bundle, mGraph));
//  mPropertiesWrite.property("ID", valueID);
//
//  auto valueNavigable = boost::make_transform_value_property_map(
//      [](Cell &cell) { return cell.GetProperty<bool>("Navigable", false); }, boost::get(boost::vertex_bundle, mGraph));
//  mPropertiesWrite.property("Navigable", valueNavigable);
//
//  auto valueXCoordinate = boost::make_transform_value_property_map(
//      [](Cell &cell) { return cell.GetProperty<unsigned int>("X", 0); }, boost::get(boost::vertex_bundle, mGraph));
//  mPropertiesWrite.property("X", valueXCoordinate);
//
//  auto valueYCoordinate = boost::make_transform_value_property_map(
//      [](Cell &cell) { return cell.GetProperty<unsigned int>("Y", 0); }, boost::get(boost::vertex_bundle, mGraph));
//  mPropertiesWrite.property("Y", valueYCoordinate);
//
//  auto valueOccupancy = boost::make_transform_value_property_map(
//      [](Cell &cell) { return cell.GetProperty<bool>("Occupancy", false); }, boost::get(boost::vertex_bundle, mGraph));
//  mPropertiesWrite.property("Occupancy", valueOccupancy);
//
//  std::ofstream fout(mGraphFilePath.c_str());
//  boost::write_graphviz_dp(fout, mGraph, mPropertiesWrite, "ID");

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Map::Graph Map::GetGraph() {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return mGraph;
}

Map::Map(std::string pGraphFilePath)
    : mLogger(log4cpp::Category::getInstance("Map")),
      mGraph(0),
      mVertex(),
      mPropertiesRead(boost::ignore_other_properties),
      mPropertiesWrite(),
      mGrid(),
      mGraphFilePath(pGraphFilePath) {
  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
