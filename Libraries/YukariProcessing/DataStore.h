/** @file */

#pragma once

#include <iostream>
#include <map>

#include <YukariCommon/LoggingService.h>
#include <boost/regex.hpp>

#include "IAlgorithm.h"
#include "Property.h"

namespace Yukari
{
namespace Processing
{
  class DataStore : public IAlgorithm::PropertyContainer
  {
  public:
    typedef std::pair<std::string, Property_sptr> Item;
    typedef std::vector<Item> ItemList;

  public:
    DataStore();

    ItemList findByRegex(const std::string &regexStr);
    ItemList findByRegex(boost::regex regex);

    void prettyPrint(std::ostream &s) const;

    friend std::ostream &operator<<(std::ostream &s, const DataStore &p);

  private:
    Common::LoggingService::Logger m_logger;
  };

  typedef std::shared_ptr<DataStore> DataStore_sptr;
}
}
