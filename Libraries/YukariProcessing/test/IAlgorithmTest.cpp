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
        m_validator = [](const IAlgorithm &alg) {
          auto a = alg.getProperty(Processing::INPUT, "a");
          auto b = alg.getProperty(Processing::INPUT, "b");

          size_t na = a.size();
          size_t nb = b.size();

          if (na != nb)
            return "Input property lengths must match";

          return "";
        };
      }

    protected:
      virtual void doExecute() override
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

    BOOST_AUTO_TEST_CASE(IAlgorithm_Validation_Algorithm)
    {
      MockAdditionAlgorithm alg;

      /* Test validation */
      {
        BOOST_CHECK(!alg.isValid());

        auto result = alg.validate();
        BOOST_CHECK_EQUAL(result.size(), 1);
        BOOST_CHECK_EQUAL(result["algorithm_validation"],
                          "Validation exception: Property \"a\" not found");
      }

      /* Set input properties */
      Property a({2, 4, 6});
      Property b({20, 21});

      /* Parameter validators */
      {
        auto validator = [](const Property::ValueStorageType &values) {
          if (values.size() < 5)
            return "Not enough values";

          return "";
        };

        a.setValidator(validator);
        b.setValidator(validator);
      }

      alg.setProperty(INPUT, "a", a);
      alg.setProperty(INPUT, "b", b);

      /* Test validation */
      {
        BOOST_CHECK(!alg.isValid());

        auto result = alg.validate();
        BOOST_CHECK_EQUAL(result.size(), 3);
        BOOST_CHECK_EQUAL(result["a"], "Not enough values");
        BOOST_CHECK_EQUAL(result["b"], "Not enough values");
        BOOST_CHECK_EQUAL(result["algorithm_validation"], "Input property lengths must match");
      }

      /* Make algorithm valid */
      {
        b.resize(3);
        b[2] = 23;
        alg.setProperty(INPUT, "b", b);
      }

      /* Test validation */
      {
        BOOST_CHECK(!alg.isValid());

        auto result = alg.validate();
        BOOST_CHECK_EQUAL(result.size(), 2);
        BOOST_CHECK_EQUAL(result["a"], "Not enough values");
        BOOST_CHECK_EQUAL(result["b"], "Not enough values");
      }

      /* Make properties valid */
      {
        a.resize(5);
        b.resize(5);

        a[3] = 8;
        a[4] = 10;

        b[3] = 25;
        b[4] = 26;

        alg.setProperty(INPUT, "a", a);
        alg.setProperty(INPUT, "b", b);
      }

      /* Test validation */
      {
        BOOST_CHECK(alg.isValid());

        auto result = alg.validate();
        BOOST_CHECK_EQUAL(result.size(), 0);
      }
    }
  }
}
}

#endif
