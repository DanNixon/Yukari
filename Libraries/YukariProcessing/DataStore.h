/** @file */

#pragma once

#include <map>

#include "YukariCommon/LoggingService.h"

#include "IAlgorithm.h"
#include "Property.h"

namespace Yukari
{
namespace Processing
{
  class DataStore : public IAlgorithm::PropertyContainer
  {
  public:
    DataStore();

    friend std::ostream &operator<<(std::ostream &s, const DataStore &p);

  private:
    Common::LoggingService::Logger m_logger;
  };

  typedef std::shared_ptr<DataStore> DataStore_sptr;
}
}
