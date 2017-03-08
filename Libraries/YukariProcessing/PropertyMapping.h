/** @file */

#pragma once

#include <memory>

#include <YukariCommon/LoggingService.h>

#include "DataStore.h"
#include "IAlgorithm.h"

namespace Yukari
{
namespace Processing
{
  struct PropertyMappingItem
  {
    PropertyDirection direction;
    std::string propertyName;
    std::string dataName;

    friend std::ostream &operator<<(std::ostream &s, const PropertyMappingItem &p)
    {
      s << "Mapping(";
      switch (p.direction)
      {
      case INPUT:
        s << p.dataName << " -> [" << p.propertyName << "]";
        break;
      case OUTPUT:
        s << "[" << p.propertyName << "] -> " << p.dataName;
        break;
      default:
        s << "unknown";
        break;
      }
      s << ")";

      return s;
    }
  };

  class PropertyMapping : public std::vector<PropertyMappingItem>
  {
  public:
    PropertyMapping();
    virtual ~PropertyMapping();

    bool parseString(const std::string &s);
    size_t parseString(std::vector<std::string>::const_iterator begin,
                       std::vector<std::string>::const_iterator end);

    void setInputs(DataStore_sptr &store, IAlgorithm_sptr alg);
    void setOutputs(DataStore_sptr &store, IAlgorithm_sptr alg);

  private:
    Common::LoggingService::Logger m_logger;
  };

  std::ostream &operator<<(std::ostream &s, const PropertyMapping &p);

  typedef std::shared_ptr<PropertyMapping> PropertyMapping_sptr;
}
}
