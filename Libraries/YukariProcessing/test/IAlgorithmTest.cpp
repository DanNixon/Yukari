/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "IAlgorithmTest"

#include <boost/test/unit_test.hpp>

#include <YukariProcessing/IAlgorithm.h>

namespace Yukari
{
namespace Processing
{
  namespace Test
  {
    class MockAdditionAlgorithm : public IAlgorithm
    {
    public:
      MockAdditionAlgorithm()
      {
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

      virtual void execute() override
      {
        size_t num = m_inputProperties["a"].size();
        m_outputProperties["z"] = Property(num);

        for (size_t i = 0; i < num; i++)
          m_outputProperties["z"][i] =
              m_inputProperties["a"].value<int>(i) + m_inputProperties["b"].value<int>(i);
      }
    };

    BOOST_AUTO_TEST_CASE(IAlgorithm_Empty)
    {
      MockAdditionAlgorithm alg;
      BOOST_CHECK(!alg.isValid());
    }

    BOOST_AUTO_TEST_CASE(IAlgorithm_Execute)
    {
      MockAdditionAlgorithm alg;

      alg.setProperty(INPUT, "a", Property({2, 4, 6, 8, 10}));
      alg.setProperty(INPUT, "b", Property({20, 21, 22, 23, 24}));

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property results = alg.getProperty(OUTPUT, "z");

      BOOST_CHECK_EQUAL(results.size(), 5);

      BOOST_CHECK_EQUAL(results.value<int>(0), 22);
      BOOST_CHECK_EQUAL(results.value<int>(1), 25);
      BOOST_CHECK_EQUAL(results.value<int>(2), 28);
      BOOST_CHECK_EQUAL(results.value<int>(3), 31);
      BOOST_CHECK_EQUAL(results.value<int>(4), 34);
    }
  }
}
}

#endif
