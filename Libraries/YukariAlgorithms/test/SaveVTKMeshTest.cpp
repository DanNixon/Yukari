/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/AlgorithmFactory.h>
#include <YukariAlgorithms/SaveVTKMesh.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(SaveVTKMeshTest)

    BOOST_AUTO_TEST_CASE(SaveVTKMesh_Create)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("SaveVTKMesh");
      BOOST_CHECK(alg);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
