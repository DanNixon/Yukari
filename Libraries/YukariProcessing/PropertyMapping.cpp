/** @file */

#include "PropertyMapping.h"

#include <boost/algorithm/string/regex.hpp>

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
    PropertyMappingItem mapping;
    boost::cmatch what;

    boost::regex inputPropRegex("(\\w+)\\s*->\\s*\\[(\\w+)\\]");
    if (boost::regex_match(s.c_str(), what, inputPropRegex))
    {
      mapping.direction = INPUT;
      mapping.dataName = what[1];
      mapping.propertyName = what[2];

      push_back(mapping);
      return true;
    }

    boost::regex outputPropRegex("\\[(\\w+)\\]\\s*->\\s*(\\w+)");
    if (boost::regex_match(s.c_str(), what, outputPropRegex))
    {
      mapping.direction = OUTPUT;
      mapping.propertyName = what[1];
      mapping.dataName = what[2];

      push_back(mapping);
      return true;
    }

    m_logger->error("Failed to parse string: \"{}\"", s);
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

  void PropertyMapping::setInputs(DataStore_sptr &store, IAlgorithm_sptr alg)
  {
    for (auto it = begin(); it != end(); ++it)
    {
      if (it->direction == INPUT)
      {
        m_logger->debug("Setting property mapping: {}", *it);
        if (store->contains(it->dataName))
          alg->setProperty(INPUT, it->propertyName, store->at(it->dataName));
        else
          m_logger->error("Data not found in data store: {}", it->dataName);
      }
    }
  }

  void PropertyMapping::setOutputs(DataStore_sptr &store, IAlgorithm_sptr alg)
  {
    for (auto it = begin(); it != end(); ++it)
    {
      if (it->direction == OUTPUT)
      {
        m_logger->debug("Setting property mapping: {}", *it);
        (*store)[it->dataName] = alg->getProperty(OUTPUT, it->propertyName);
      }
    }
  }

  std::ostream &operator<<(std::ostream &s, const PropertyMapping &p)
  {
    std::stringstream inputProperties;
    std::stringstream outputProperties;

    for (auto it = p.cbegin(); it != p.cend(); ++it)
    {
      if (it->direction == INPUT)
        inputProperties << it->dataName << "->" << it->propertyName << " ";
      else if (it->direction == OUTPUT)
        inputProperties << it->propertyName << "->" << it->dataName << " ";
    }

    s << "PropertyMapping[Input[" << inputProperties.str() << "], Output[" << outputProperties.str()
      << "]]";

    return s;
  }
}
}
