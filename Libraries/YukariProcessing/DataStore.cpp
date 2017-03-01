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

  DataStore::ItemList DataStore::findByRegex(const std::string &regexStr)
  {
    return findByRegex(boost::regex(regexStr));
  }

  DataStore::ItemList DataStore::findByRegex(boost::regex regex)
  {
    DataStore::ItemList retVal;

    for (auto it = begin(); it != end(); ++it)
    {
      boost::cmatch what;
      if (boost::regex_match(it->first.c_str(), what, regex))
        retVal.push_back(*it);
    }

    return retVal;
  }

  void DataStore::prettyPrint(std::ostream &s) const
  {
    for (auto it = begin(); it != end(); ++it)
      s << it->first << " (" << it->second->size() << ")\n";
  }

  std::ostream &operator<<(std::ostream &s, const DataStore &p)
  {
    s << "DataStore[size=" << p.size() << "]";
    return s;
  }
}
}
