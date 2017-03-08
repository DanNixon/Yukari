/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/Add.h>
#include <YukariAlgorithms/AlgorithmFactory.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(AddTest)

    BOOST_AUTO_TEST_CASE(Add_Create)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("Add");
      BOOST_CHECK(alg);
    }

    BOOST_AUTO_TEST_CASE(Add_Execute)
    {
      Add alg;

      alg.setProperty(Processing::INPUT, "a", Property_sptr(new Property({2, 4, 6, 8, 10})));
      alg.setProperty(Processing::INPUT, "b", Property_sptr(new Property({20, 21, 22, 23, 24})));

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property_sptr results = alg.getProperty(OUTPUT, "z");

      BOOST_CHECK_EQUAL(results->size(), 5);

      BOOST_CHECK_EQUAL(results->value<int>(0), 22);
      BOOST_CHECK_EQUAL(results->value<int>(1), 25);
      BOOST_CHECK_EQUAL(results->value<int>(2), 28);
      BOOST_CHECK_EQUAL(results->value<int>(3), 31);
      BOOST_CHECK_EQUAL(results->value<int>(4), 34);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
