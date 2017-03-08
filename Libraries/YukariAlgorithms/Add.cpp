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
    m_validator = [](const IAlgorithm &alg) {
      auto a = alg.getProperty(Processing::INPUT, "a");
      auto b = alg.getProperty(Processing::INPUT, "b");

      size_t na = a->size();
      size_t nb = b->size();

      if (na != nb)
        return "Input property lengths must match";

      return "";
    };
  }

  void Add::doExecute()
  {
    Property_sptr a = getProperty(Processing::INPUT, "a");
    Property_sptr b = getProperty(Processing::INPUT, "b");

    size_t len = a->size();
    Property_sptr z = std::make_shared<Property>(len);

    for (size_t i = 0; i < len; i++)
      (*z)[i] = a->value<int>(i) + b->value<int>(i);

    setProperty(Processing::OUTPUT, "z", z);
  }
}
}
