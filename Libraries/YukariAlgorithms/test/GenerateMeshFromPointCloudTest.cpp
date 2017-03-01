/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "GenerateMeshFromPointCloudTest"

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/GenerateMeshFromPointCloud.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(GenerateMeshFromPointCloud_Execute)
    {
      GenerateMeshFromPointCloud alg;

      /* TODO */

      BOOST_CHECK(alg.isValid());

      alg.execute();
    }
  }
}
}

#endif
