// Copyright [2015] Avantys Engineering GmbH & CoKG

#ifndef MAP_MANAGEMENT_CELL_H_
#define MAP_MANAGEMENT_CELL_H_

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/CellProperty.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <log4cpp/Category.hh>
#include <log4cpp/BasicConfigurator.hh>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <string>

namespace accmetnavigation {
/*!
 */
class Cell {
 public:
  Cell();
  virtual ~Cell();

  Cell& operator=(Cell pSourceCell);

  template <class T>
  T GetProperty(std::string pPropertyName, T pDefaultValue = T()) {
//    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
    try {
      T propertyValue = boost::lexical_cast<T>(
          mCellProperty.GetPropertyString(pPropertyName, boost::lexical_cast<std::string>(pDefaultValue)));
      return propertyValue;
    } catch (...) {
      mLogger << log4cpp::Priority::DEBUG << __func__
              << ":  Unknown exception thrown while getting cell property value :'" << pPropertyName << "'";
    }
//    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return pDefaultValue;
  }

  template <class T>
  bool SetProperty(std::string pPropertyName, T pValue) {
//    mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";

    bool retVal = false;
    try {
      retVal = mCellProperty.SetPropertyString(pPropertyName, boost::lexical_cast<std::string>(pValue));
    } catch (...) {
      mLogger << log4cpp::Priority::DEBUG << __func__ << ":  Unknown exception thrown while writing cell property :'"
              << pPropertyName << "'";
    }
//    mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
    return retVal;
  }

 private:
  log4cpp::Category& mLogger;
  CellProperty mCellProperty;
};

} /* namespace accmetnavigation */
#endif  // MAP_MANAGEMENT_CELL_H_
