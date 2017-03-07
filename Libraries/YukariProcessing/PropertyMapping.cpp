/** @file */

#include "PropertyMapping.h"

namespace Yukari
{
namespace Processing
{
  PropertyMapping::PropertyMapping()
      : m_logger(Common::LoggingService::GetLogger("PropertyMapping"))
  {
  }

  PropertyMapping::~PropertyMapping()
  {
  }

  bool PropertyMapping::parseString(const std::string &s)
  {
    // TODO
    return false;
  }

  size_t PropertyMapping::parseString(std::vector<std::string>::const_iterator begin,
                                      std::vector<std::string>::const_iterator end)
  {
    size_t i = 0;

    for (; begin != end; ++begin)
    {
      if (parseString(*begin))
        i++;
    }

    return i;
  }
}
}
