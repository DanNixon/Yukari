/** @file */

#pragma once

#include <functional>
#include <iostream>
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
    Property(size_t size = 1)
    {
      m_values.resize(size);
    }

    Property(const ValueStorageType &values)
        : m_values(values)
    {
    }

    virtual ~Property()
    {
      m_values.clear();
    }

    inline size_t size() const
    {
      return m_values.size();
    }

    void setValidator(Validator validator)
    {
      m_validator = validator;
    }

    void removeValidator()
    {
      m_validator = Validator();
    }

    bool validate() const
    {
      return (m_validator ? m_validator(m_values) : true);
    }

    template <typename T> T value(size_t idx = 0) const
    {
      if (idx >= m_values.size())
        throw std::runtime_error("Property index out of range");

      return boost::any_cast<T>(m_values[idx]);
    }

    boost::any operator[](size_t idx) const
    {
      if (idx >= m_values.size())
        throw std::runtime_error("Property index out of range");

      return m_values[idx];
    }

    boost::any &operator[](size_t idx)
    {
      if (idx >= m_values.size())
        throw std::runtime_error("Property index out of range");

      return m_values[idx];
    }

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

  protected:
    ValueStorageType m_values;
    Validator m_validator;
  };

  typedef std::shared_ptr<Property> Property_sptr;

  std::ostream &operator<<(std::ostream &s, Property &o)
  {
    s << "Property[size=" << o.size() << "]";
    return s;
  }
}
}
