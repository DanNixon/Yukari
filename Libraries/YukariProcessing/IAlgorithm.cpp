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
        std::string pvRes = pIt->second->validate();
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
      std::string algValidationRes;
      try
      {
        algValidationRes = m_validator(*this);
      }
      catch (std::runtime_error &e)
      {
        algValidationRes = "Validation exception: " + std::string(e.what());
      }
      catch (...)
      {
        algValidationRes = "Unknown validation exception";
      }

      if (!algValidationRes.empty())
      {
        m_algBaseLogger->warn("Algorithm specific validation failed: {}", algValidationRes);
        res["algorithm_validation"] = algValidationRes;
      }
    }

    return res;
  }

  bool IAlgorithm::setProperty(PropertyDirection dir, const std::string &name, Property_sptr prop)
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

  Property_sptr IAlgorithm::getProperty(PropertyDirection dir, const std::string &name) const
  {
    m_algBaseLogger->trace("Requested property \"{}\"", name);

    /* Get property storage */
    const PropertyContainer *c = nullptr;
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

    /* Check if property was found */
    auto it = c->find(name);
    if (it == c->cend())
    {
      throw std::runtime_error("Property \"" + name + "\" not found");
    }

    return it->second;
  }

  void IAlgorithm::execute()
  {
    try
    {
      doExecute();
    }
    catch (std::runtime_error &e)
    {
      m_algBaseLogger->error("Algorithm execution failed with exception: {}", e.what());
    }
    catch (...)
    {
      m_algBaseLogger->error("Algorithm execution failed with unknown exception");
    }
  }
}
}
