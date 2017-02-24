/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "AddTest"

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/Add.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(Add_Execute)
    {
      Add alg;

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
