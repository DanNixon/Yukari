/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/AlgorithmFactory.h>
#include <YukariAlgorithms/LoadPointCloud.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(LoadPointCloudTest)

    BOOST_AUTO_TEST_CASE(LoadPointCloud_Create)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("LoadPointCloud");
      BOOST_CHECK(alg);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
