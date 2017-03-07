/** @file */

#pragma once

#include <map>
#include <memory>

#include <YukariCommon/LoggingService.h>

#include "Property.h"

namespace Yukari
{
namespace Processing
{
  enum PropertyDirection
  {
    INPUT,
    OUTPUT
  };

  class IAlgorithm
  {
  public:
    typedef std::map<std::string, Property_sptr> PropertyContainer;
    typedef std::map<std::string, std::string> ValidationResults;
    typedef std::function<std::string(const IAlgorithm &alg)> Validator;

  public:
    IAlgorithm();
    virtual ~IAlgorithm();

    inline void setValidator(Validator validator)
    {
      m_validator = validator;
    }

    inline void removeValidator()
    {
      m_validator = Validator();
    }

    ValidationResults validate() const;

    inline bool isValid() const
    {
      return validate().empty();
    }

    bool setProperty(PropertyDirection dir, const std::string &name, Property_sptr prop);
    Property_sptr getProperty(PropertyDirection dir, const std::string &name) const;

    void execute();

  protected:
    virtual void doExecute() = 0;

  protected:
    Validator m_validator;
    PropertyContainer m_inputProperties;
    PropertyContainer m_outputProperties;

  private:
    Common::LoggingService::Logger m_algBaseLogger;
  };

  typedef std::shared_ptr<IAlgorithm> IAlgorithm_sptr;
}
}
