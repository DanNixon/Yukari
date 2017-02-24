/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "IMUFrameToEigenTransformationTest"

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/IMUFrameToEigenTransformation.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(IMUFrameToEigenTransformation_Execute)
    {
      IMUFrameToEigenTransformation alg;

      /* alg.setProperty(INPUT, "a", Property({2, 4, 6, 8, 10})); */
      /* alg.setProperty(INPUT, "b", Property({20, 21, 22, 23, 24})); */

      /* BOOST_CHECK(alg.isValid()); */

      /* alg.execute(); */

      /* Property results = alg.getProperty(OUTPUT, "z"); */

      /* BOOST_CHECK_EQUAL(results.size(), 5); */

      /* BOOST_CHECK_EQUAL(results.value<int>(0), 31); */
    }
  }
}
}

#endif
