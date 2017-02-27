/** @file */

#include "IAlgorithm.h"

namespace Yukari
{
namespace Processing
{
  IAlgorithm::IAlgorithm()
      : m_algBaseLogger(Common::LoggingService::GetLogger("IAlgorithm"))
  {
  }

  IAlgorithm::~IAlgorithm()
  {
  }

  IAlgorithm::ValidationResults IAlgorithm::validate() const
  {
    ValidationResults res;

    /* Iterate over input and output property containers */
    auto props = {&m_inputProperties, &m_outputProperties};
    for (auto cIt = props.begin(); cIt != props.end(); ++cIt)
    {
      /* Iterate over properties */
      for (auto pIt = (*cIt)->begin(); pIt != (*cIt)->end(); ++pIt)
      {
        /* Validate property */
        std::string pvRes = pIt->second.validate();
        if (!pvRes.empty())
        {
          /* Record validation failure */
          m_algBaseLogger->warn("Property \"{}\" failed validation: {}", pIt->first, pvRes);
          res[pIt->first] = pvRes;
        }
      }
    }

    /* Perform algorithm specific validation */
    if (m_validator)
    {
      std::string algValidationRes = m_validator(m_inputProperties, m_outputProperties);
      if (!algValidationRes.empty())
      {
        m_algBaseLogger->warn("Algorithm specific validation failed: {}", algValidationRes);
        res["algorithm_validation"] = algValidationRes;
      }
    }

    return res;
  }

  bool IAlgorithm::setProperty(PropertyDirection dir, const std::string &name, Property prop)
  {
    /* Get property storage */
    PropertyContainer *c = nullptr;
    switch (dir)
    {
    case INPUT:
      c = &m_inputProperties;
      break;
    case OUTPUT:
      c = &m_outputProperties;
      break;
    default:
      m_algBaseLogger->error("Unknown property direction");
      return false;
    }

    if (c == nullptr)
    {
      m_algBaseLogger->error("Could not get property storage");
      return false;
    }

    /* Set the new property */
    (*c)[name] = prop;
    m_algBaseLogger->trace("Set property \"{}\"", name);
    return true;
  }

  Property IAlgorithm::getProperty(PropertyDirection dir, const std::string &name)
  {
    m_algBaseLogger->trace("Requested property \"{}\"", name);

    switch (dir)
    {
    case INPUT:
      return m_inputProperties[name];
    case OUTPUT:
      return m_outputProperties[name];
    default:
      throw std::runtime_error("Unknown property direction");
    }
  }
}
}
