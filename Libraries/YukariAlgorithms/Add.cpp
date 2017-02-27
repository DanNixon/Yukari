/** @file */

#include "Add.h"

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  Add::Add()
  {
    /* Add default validator */
    m_validator = [](const PropertyContainer &inProps, const PropertyContainer &) {
      auto a = inProps.find("a");
      if (a == inProps.end())
        return "Input property \"a\" not found";

      auto b = inProps.find("b");
      if (b == inProps.end())
        return "Input property \"b\" not found";

      size_t na = a->second.size();
      size_t nb = b->second.size();

      if (na != nb)
        return "Input property lengths must match";

      return "";
    };
  }

  void Add::execute()
  {
    Property a = getProperty(Processing::INPUT, "a");
    Property b = getProperty(Processing::INPUT, "b");
    size_t len = a.size();
    Property z(len);

    for (size_t i = 0; i < len; i++)
      z[i] = a.value<int>(i) + b.value<int>(i);

    setProperty(Processing::OUTPUT, "z", z);
  }
}
}
