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

  std::ostream &operator<<(std::ostream &s, const DataStore &p)
  {
    s << "DataStore[size=" << p.size() << "]";
    return s;
  }
}
}
