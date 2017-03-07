/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "ConcatenatePointCloudsTest"

#include <YukariCloudCapture/ICloudGrabber.h>
#include <boost/test/unit_test.hpp>

#include <YukariAlgorithms/AlgorithmFactory.h>
#include <YukariAlgorithms/ConcatenatePointClouds.h>

using namespace Yukari::CloudCapture;
using namespace Yukari::Processing;

namespace Yukari
{
namespace Algorithms
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(ConcatenatePointClouds_Create)
    {
      IAlgorithm_sptr alg = AlgorithmFactory::Create("ConcatenatePointClouds");
      BOOST_CHECK(alg);
    }

    BOOST_AUTO_TEST_CASE(ConcatenatePointClouds_Execute)
    {
      ConcatenatePointClouds alg;

      Property_sptr clouds = std::make_shared<Property>(5);

      for (size_t i = 0; i < clouds->size(); i++)
      {
        ICloudGrabber::Cloud::Ptr c(new ICloudGrabber::Cloud());

        c->width = 1;
        c->height = 1;
        c->is_dense = false;
        c->points.resize(1);

        c->points[0].x = (float)i;
        c->points[0].y = (float)i;
        c->points[0].z = (float)i;

        c->points[0].rgba = 0xFFFFFFFF;

        (*clouds)[i] = c;
      }

      alg.setProperty(Processing::INPUT, "clouds", clouds);

      BOOST_CHECK(alg.isValid());

      alg.execute();

      Property_sptr results = alg.getProperty(OUTPUT, "cloud");
      BOOST_CHECK_EQUAL(results->size(), 1);

      ICloudGrabber::Cloud::Ptr resultCloud = results->value<ICloudGrabber::Cloud::Ptr>(0);
      BOOST_CHECK_EQUAL(resultCloud->width, 5);
      BOOST_CHECK_EQUAL(resultCloud->height, 1);
      BOOST_CHECK_EQUAL(resultCloud->points.size(), clouds->size());

      BOOST_CHECK_EQUAL(resultCloud->points[0].x, 0.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[1].x, 1.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[2].x, 2.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[3].x, 3.0f);
      BOOST_CHECK_EQUAL(resultCloud->points[4].x, 4.0f);
    }
  }
}
}

#endif
