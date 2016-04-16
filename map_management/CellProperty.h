// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MAP_MANAGEMENT_CELLPROPERTY_H_
#define MAP_MANAGEMENT_CELLPROPERTY_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <string>

namespace accmetnavigation {
/*!
 */
class CellProperty {
 public:
  CellProperty();
  virtual ~CellProperty();

  CellProperty& operator=(CellProperty pSourceProperty);

  std::string GetPropertyString(std::string pName, std::string pDefaultValue);
  bool SetPropertyString(std::string pName, std::string pValue);

 private:
  log4cpp::Category& mLogger;
  std::string mID;
  std::string mNavigable;
  std::string mXCoordinate;
  std::string mYCoordinate;
  std::string mOccupancy;
};

} /* namespace accmetnavigation */
#endif  // MAP_MANAGEMENT_CELLPROPERTY_H_
