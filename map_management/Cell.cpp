// Copyright [2015] Avantys Engineering GmbH & CoKG

//-----------------------------------------------------------------------------
// Project Includes
//-----------------------------------------------------------------------------
#include <map_management/Cell.h>

//-----------------------------------------------------------------------------
// Global Includes
//-----------------------------------------------------------------------------

namespace accmetnavigation {

Cell::Cell() : mLogger(log4cpp::Category::getInstance("Cell")), mCellProperty() {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

Cell& Cell::operator=(Cell pSourceCell) {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
  if (this != &pSourceCell) {
    mCellProperty = pSourceCell.mCellProperty;
  }
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
  return *this;
}

Cell::~Cell() {
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": ENTRY ";
//  mLogger << log4cpp::Priority::DEBUG << __func__ << ": EXIT ";
}

} /* namespace accmetnavigation */
