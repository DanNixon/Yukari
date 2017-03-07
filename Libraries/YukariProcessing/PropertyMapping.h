/** @file */

#pragma once

#include <memory>

#include <YukariCommon/LoggingService.h>

#include "IAlgorithm.h"

namespace Yukari
{
namespace Processing
{
  class PropertyMapping
  {
  public:
    struct Mapping
    {
      PropertyDirection direction;
      std::string propertyName;
      std::string dataName;
    };

  public:
    PropertyMapping();
    virtual ~PropertyMapping();

    bool parseString(const std::string &s);
    size_t parseString(std::vector<std::string>::const_iterator begin,
                       std::vector<std::string>::const_iterator end);

  protected:
    std::vector<Mapping> m_mappings;

  private:
    Common::LoggingService::Logger m_logger;
  };

  typedef std::shared_ptr<PropertyMapping> PropertyMapping_sptr;
}
}
