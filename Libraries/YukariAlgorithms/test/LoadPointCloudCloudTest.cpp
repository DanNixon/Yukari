/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "LoadPointCloudTest"

#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/LoadPointCloud.h>

using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(LoadPointCloud_Execute)
    {
      LoadPointCloud alg;

      Property file({std::string("")});

      alg.setProperty(Processing::INPUT, "file", file);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property results = alg.getProperty(Processing::OUTPUT, "cloud");
      // BOOST_CHECK_EQUAL(results.size(), file.size());

      // TODO
    }
  }
}
}

#endif
