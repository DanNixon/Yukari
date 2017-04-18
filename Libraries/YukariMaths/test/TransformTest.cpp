/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>
#include <sstream>

#include <YukariMaths/Quaternion.h>
#include <YukariMaths/Transform.h>
#include <YukariMaths/Vector3.h>

using namespace boost::qvm;
namespace po = boost::program_options;

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(IMUFrameTest)

    BOOST_AUTO_TEST_CASE(Transform_Stream_Out_Empty)
    {
      Transform t;

      std::stringstream str;
      str << t;

      BOOST_CHECK_EQUAL(str.str(), "(o=[1, 0, 0, 0], p=[0, 0, 0])");
    }

    BOOST_AUTO_TEST_CASE(Transform_Stream_Out)
    {
      Transform t;
      t.orientation() = Quaternion(Vector3(30.0f, 30.0f, 30.0f), Quaternion::ZYX, DEGREES);
      t.position() = Vector3(5.5f, 7.8f, 2.1f);

      std::stringstream str;
      str << t;

      BOOST_CHECK_EQUAL(str.str(),
                        "(o=[0.918559, 0.176777, 0.306186, 0.176777], p=[5.5, 7.8, 2.1])");
    }

    BOOST_AUTO_TEST_CASE(Transform_From_Boost_Args_Orientation_Only)
    {
      po::variables_map args;
      args.insert(std::make_pair(
          "orientation", po::variable_value(boost::any(std::string("[0, 0, 1] -90.5")), false)));

      Transform t(args);

      Quaternion expectedQuat(Vector3(0.0f, 0.0f, 1.0f), -90.5f, DEGREES);
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.i(), t.orientation().i());
      BOOST_CHECK_EQUAL(expectedQuat.j(), t.orientation().j());
      BOOST_CHECK_EQUAL(expectedQuat.k(), t.orientation().k());

      BOOST_CHECK_EQUAL(t.position().x(), 0.0f);
      BOOST_CHECK_EQUAL(t.position().y(), 0.0f);
      BOOST_CHECK_EQUAL(t.position().z(), 0.0f);
    }

    BOOST_AUTO_TEST_CASE(Transform_From_Boost_Args_Position_Only)
    {
      po::variables_map args;
      args.insert(std::make_pair(
          "position", po::variable_value(boost::any(std::string("[1, 4.5, 7.1]")), false)));

      Transform t(args);

      Quaternion expectedQuat;
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.i(), t.orientation().i());
      BOOST_CHECK_EQUAL(expectedQuat.j(), t.orientation().j());
      BOOST_CHECK_EQUAL(expectedQuat.k(), t.orientation().k());

      BOOST_CHECK_EQUAL(t.position().x(), 1.0f);
      BOOST_CHECK_EQUAL(t.position().y(), 4.5f);
      BOOST_CHECK_EQUAL(t.position().z(), 7.1f);
    }

    BOOST_AUTO_TEST_CASE(Transform_From_Boost_Args_Both)
    {
      po::variables_map args;
      args.insert(std::make_pair(
          "orientation", po::variable_value(boost::any(std::string("[0, 0, 1] 90.0")), false)));
      args.insert(std::make_pair(
          "position", po::variable_value(boost::any(std::string("[1, 4.5, 7.1]")), false)));

      Transform t(args);

      Quaternion expectedQuat(Vector3(0.0f, 0.0f, 1.0f), 90.0f, DEGREES);
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.i(), t.orientation().i());
      BOOST_CHECK_EQUAL(expectedQuat.j(), t.orientation().j());
      BOOST_CHECK_EQUAL(expectedQuat.k(), t.orientation().k());

      BOOST_CHECK_EQUAL(t.position().x(), 1.0f);
      BOOST_CHECK_EQUAL(t.position().y(), 4.5f);
      BOOST_CHECK_EQUAL(t.position().z(), 7.1f);
    }

    BOOST_AUTO_TEST_CASE(Transform_From_Boost_Args_Both_Empty)
    {
      po::variables_map args;
      args.insert(
          std::make_pair("orientation", po::variable_value(boost::any(std::string("")), false)));
      args.insert(
          std::make_pair("position", po::variable_value(boost::any(std::string("")), false)));

      Transform t(args);

      Quaternion expectedQuat;
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.i(), t.orientation().i());
      BOOST_CHECK_EQUAL(expectedQuat.j(), t.orientation().j());
      BOOST_CHECK_EQUAL(expectedQuat.k(), t.orientation().k());

      BOOST_CHECK_EQUAL(t.position().x(), 0.0f);
      BOOST_CHECK_EQUAL(t.position().y(), 0.0f);
      BOOST_CHECK_EQUAL(t.position().z(), 0.0f);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
