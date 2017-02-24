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
    typedef std::function<std::string(const ValueStorageType &props)> Validator;

  public:
    Property(size_t size = 1)
    {
      m_values.resize(size);
    }

    Property(const ValueStorageType &values)
        : m_values(values)
    {
    }

    Property(const Property &other)
        : m_values(other.m_values)
        , m_validator(other.m_validator)
    {
    }

    virtual ~Property()
    {
      m_values.clear();
    }

    inline void resize(size_t size)
    {
      m_values.resize(size);
    }

    inline size_t size() const
    {
      return m_values.size();
    }

    inline void setValidator(Validator validator)
    {
      m_validator = validator;
    }

    inline void removeValidator()
    {
      m_validator = Validator();
    }

    inline std::string validate() const
    {
      return (m_validator ? m_validator(m_values) : "");
    }

    inline bool isValid() const
    {
      return validate().empty();
    }

    template <typename T> bool isA(size_t idx = 0) const
    {
      if (idx >= m_values.size())
        throw std::runtime_error("Property index out of range");

      return (boost::any_cast<T>(m_values[idx]));
    }

    template <typename T> T value(size_t idx = 0) const
    {
      if (idx >= m_values.size())
        throw std::runtime_error("Property index out of range");

      return boost::any_cast<T>(m_values[idx]);
    }

    inline boost::any operator[](size_t idx) const
    {
      if (idx >= m_values.size())
        throw std::runtime_error("Property index out of range");

      return m_values[idx];
    }

    inline boost::any &operator[](size_t idx)
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

    friend std::ostream &operator<<(std::ostream &s, Property &o)
    {
      s << "Property[size=" << o.size() << "]";
      return s;
    }

  protected:
    ValueStorageType m_values;
    Validator m_validator;
  };

  typedef std::shared_ptr<Property> Property_sptr;
}
}
