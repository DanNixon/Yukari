/** @file */

#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <boost/any.hpp>

namespace Yukari
{
namespace Processing
{
  class Property
  {
  public:
    typedef std::vector<boost::any> ValueStorageType;
    typedef std::function<bool(const ValueStorageType &)> Validator;

  public:
    Property(size_t size = 1);
    virtual ~Property();

    inline size_t size() const
    {
      return m_values.size();
    }

    void setValidator(Validator validator);
    void removeValidator();
    bool validate() const;

    boost::any value() const;

    boost::any operator[](size_t idx) const;
    boost::any &operator[](size_t idx);

    inline ValueStorageType::iterator begin()
    {
      return m_values.begin();
    }

    inline ValueStorageType::iterator end()
    {
      return m_values.end();
    }

    inline ValueStorageType::const_iterator cbegin() const
    {
      return m_values.cbegin();
    }

    inline ValueStorageType::const_iterator cend() const
    {
      return m_values.cend();
    }

    friend std::ostream &operator<<(std::ostream &s, Property &o);

  protected:
    ValueStorageType m_values;
    Validator m_validator;
  };

  typedef std::shared_ptr<Property> Property_sptr;
}
}
