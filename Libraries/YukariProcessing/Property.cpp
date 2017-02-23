/** @file */

#include "Property.h"

namespace Yukari
{
namespace Processing
{
  Property::Property(size_t size)
  {
    m_values.resize(size);
  }

  Property::~Property()
  {
    m_values.clear();
  }

  void Property::setValidator(Validator validator)
  {
    m_validator = validator;
  }

  void Property::removeValidator()
  {
    m_validator = Validator();
  }

  bool Property::validate() const
  {
    return (m_validator ? m_validator(m_values) : true);
  }

  boost::any Property::value() const
  {
    return (m_values.empty() ? boost::any() : m_values.front());
  }

  boost::any Property::operator[](size_t idx) const
  {
    if (idx >= m_values.size())
      throw std::runtime_error("Property index out of range");

    return m_values[idx];
  }

  boost::any &Property::operator[](size_t idx)
  {
    if (idx >= m_values.size())
      throw std::runtime_error("Property index out of range");

    return m_values[idx];
  }

  std::ostream &operator<<(std::ostream &s, Property &o)
  {
    s << "Property[size=" << o.size() << "]";
    return s;
  }
}
}
