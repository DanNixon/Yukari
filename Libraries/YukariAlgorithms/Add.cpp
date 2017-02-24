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
    size_t num = m_inputProperties["a"].size();
    m_outputProperties["z"] = Property(num);

    for (size_t i = 0; i < num; i++)
      m_outputProperties["z"][i] =
          m_inputProperties["a"].value<int>(i) + m_inputProperties["b"].value<int>(i);
  }
}
}
