/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "TriangulatePointCloudTest"

#include <Eigen/Geometry>
#include <YukariCloudCapture/ICloudGrabber.h>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/TriangulatePointCloud.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(TriangulatePointCloud_Execute)
    {
      const size_t len = 3;

      TriangulatePointCloud alg;

      Property_sptr cloud = std::make_shared<Property>(len);

      for (size_t i = 0; i < len; i++)
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 1;
        c->height = 1;
        c->is_dense = false;
        c->points.resize(1);

        /* TODO */

        (*cloud)[i] = c;
      }

      alg.setProperty(Processing::INPUT, "cloud", cloud);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property_sptr results = alg.getProperty(Processing::OUTPUT, "mesh");
      BOOST_CHECK_EQUAL(results->size(), len);

      /* TODO */
    }
  }
}
}

#endif
