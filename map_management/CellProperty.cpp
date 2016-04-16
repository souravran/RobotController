// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/CellProperty.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------
#include <string>

namespace accmetnavigation {

CellProperty::CellProperty()
    : mLogger(log4cpp::Category::getInstance("CellProperty")),
      mID(""),
      mNavigable(""),
      mXCoordinate(""),
      mYCoordinate("") {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

CellProperty& CellProperty::operator=(CellProperty pSourceProperty) {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if (this != &pSourceProperty) {
    mID = pSourceProperty.mID;
    mNavigable = pSourceProperty.mNavigable;
    mXCoordinate = pSourceProperty.mXCoordinate;
    mYCoordinate = pSourceProperty.mYCoordinate;
  }
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return *this;
}

std::string CellProperty::GetPropertyString(std::string pName, std::string pDefaultValue) {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  std::string propValue;
  if ((pName.compare("ID") == 0) && (!mID.empty())) {
    propValue = mID;
  } else if ((pName.compare("Navigable") == 0) && (!mNavigable.empty())) {
    propValue = mNavigable;
  } else if ((pName.compare("X") == 0) && (!mXCoordinate.empty())) {
    propValue = mXCoordinate;
  } else if ((pName.compare("Y") == 0) && (!mYCoordinate.empty())) {
    propValue = mYCoordinate;
  } else if ((pName.compare("Occupancy") == 0) && (!mOccupancy.empty())) {
    propValue = mOccupancy;
  } else {
    propValue = pDefaultValue;
  }
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return propValue;
}

bool CellProperty::SetPropertyString(std::string pName, std::string pValue) {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  bool retVal = true;
  if (pName.compare("ID") == 0) {
    mID = pValue;
  } else if (pName.compare("Navigable") == 0) {
    mNavigable = pValue;
  } else if (pName.compare("X") == 0) {
    mXCoordinate = pValue;
  } else if (pName.compare("Y") == 0) {
    mYCoordinate = pValue;
  } else if (pName.compare("Occupancy") == 0) {
    mOccupancy = pValue;
  } else {
    retVal = false;
  }
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return retVal;
}

CellProperty::~CellProperty() {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
