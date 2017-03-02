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
    typedef std::function<void(DataStore::iterator &it)> IterateFunc;
    typedef std::function<void(DataStore::const_iterator &it)> ConstIterateFunc;

    typedef std::pair<std::string, Property_sptr> Item;
    typedef std::vector<Item> ItemList;

  public:
    DataStore();

    bool contains(const std::string &name) const;
    ItemList findByRegex(boost::regex regex) const;

    std::vector<std::string> deleteByRegex(boost::regex regex);

    Property_sptr addNewProperty(const std::string &name, size_t len);

    void prettyPrint(std::ostream &s) const;
    void prettyPrint(std::ostream &s, boost::regex regex) const;

    friend std::ostream &operator<<(std::ostream &s, const DataStore &p);

  protected:
    void iterateByRegex(boost::regex regex, IterateFunc f);
    void iterateByRegex(boost::regex regex, ConstIterateFunc f) const;

  private:
    Common::LoggingService::Logger m_logger;
  };

  typedef std::shared_ptr<DataStore> DataStore_sptr;
}
}
