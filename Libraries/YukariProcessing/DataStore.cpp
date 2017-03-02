/** @file */

#include "DataStore.h"

namespace Yukari
{
namespace Processing
{
  DataStore::DataStore()
      : m_logger(Common::LoggingService::GetLogger("DataStore"))
  {
  }

  bool DataStore::contains(const std::string &name) const
  {
    return find(name) != cend();
  }

  DataStore::ItemList DataStore::findByRegex(boost::regex regex) const
  {
    DataStore::ItemList retVal;
    iterateByRegex(regex, [&retVal](DataStore::const_iterator &it) {
      retVal.push_back(*it);
      ++it;
    });
    return retVal;
  }

  std::vector<std::string> DataStore::deleteByRegex(boost::regex regex)
  {
    std::vector<std::string> retVal;
    iterateByRegex(regex, [this, &retVal](DataStore::iterator &it) {
      retVal.push_back(it->first);
      it = erase(it);
    });
    return retVal;
  }

  Property_sptr DataStore::addNewProperty(const std::string &name, size_t len)
  {
    if (find(name) != end())
    {
      m_logger->info("Property \"{}\" already exists", name);
      return Property_sptr();
    }

    m_logger->debug("Adding new property with name \"{}\" and size {}", name, len);
    Property_sptr p = std::make_shared<Property>(len);
    (*this)[name] = p;

    return p;
  }

  void DataStore::prettyPrint(std::ostream &s) const
  {
    for (auto it = begin(); it != end(); ++it)
      s << it->first << " (" << it->second->size() << ")\n";
  }

  void DataStore::prettyPrint(std::ostream &s, boost::regex regex) const
  {
    iterateByRegex(regex, [&s](DataStore::const_iterator &it) {
      s << it->first << " (" << it->second->size() << ")\n";
      ++it;
    });
  }

  std::ostream &operator<<(std::ostream &s, const DataStore &p)
  {
    s << "DataStore[size=" << p.size() << "]";
    return s;
  }

  void DataStore::iterateByRegex(boost::regex regex, IterateFunc f)
  {
    for (auto it = begin(); it != end();)
    {
      boost::cmatch what;
      if (boost::regex_match(it->first.c_str(), what, regex))
        f(it);
      else
        ++it;
    }
  }

  void DataStore::iterateByRegex(boost::regex regex, ConstIterateFunc f) const
  {
    for (auto it = cbegin(); it != cend();)
    {
      boost::cmatch what;
      if (boost::regex_match(it->first.c_str(), what, regex))
        f(it);
      else
        ++it;
    }
  }
}
}
