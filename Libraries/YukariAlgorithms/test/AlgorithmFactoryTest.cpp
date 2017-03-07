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
    BOOST_AUTO_TEST_SUITE(AlgorithmFactoryTest)

    BOOST_AUTO_TEST_CASE(AlgorithmFactory_No_Algorithm_By_Name)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("nope");
      BOOST_CHECK(!alg);
    }

    BOOST_AUTO_TEST_CASE(AlgorithmFactory_Trim)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("  loadpointcloud  ");
      BOOST_CHECK(alg);
      BOOST_CHECK(std::dynamic_pointer_cast<LoadPointCloud>(alg));
    }

    BOOST_AUTO_TEST_CASE(AlgorithmFactory_Case)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("LoAdPoInTcLoUd");
      BOOST_CHECK(alg);
      BOOST_CHECK(std::dynamic_pointer_cast<LoadPointCloud>(alg));
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
