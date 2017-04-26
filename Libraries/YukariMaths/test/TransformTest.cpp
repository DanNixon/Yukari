/** @file */

#include <boost/program_options.hpp>
#include <boost/test/unit_test.hpp>
#include <sstream>

#include <YukariMaths/Transform.h>
#include <YukariMaths/Units.h>

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

      BOOST_CHECK_EQUAL(str.str(), "(o=[0, 0, 0, 1], p=[0, 0, 0])");
    }

    BOOST_AUTO_TEST_CASE(Transform_Stream_Out)
    {
      Transform t;
      t.orientation() = Eigen::AngleAxisf(30.0f * DEG_TO_RAD, Eigen::Vector3f::UnitZ()) *
                        Eigen::AngleAxisf(30.0f * DEG_TO_RAD, Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxisf(30.0f * DEG_TO_RAD, Eigen::Vector3f::UnitX());
      t.position() = Eigen::Vector3f(5.5f, 7.8f, 2.1f);

      std::stringstream str;
      str << t;

      BOOST_CHECK_EQUAL(str.str(),
                        "(o=[0.176777, 0.306186, 0.176777, 0.918559], p=[5.5, 7.8, 2.1])");
    }

    BOOST_AUTO_TEST_CASE(Transform_Stream_In)
    {
      Transform t;

      std::stringstream in("(o=[0.176777, 0.306186, 0.176777, 0.918559], p=[5.5, 7.8, 2.1])");
      in >> t;

      std::stringstream out;
      out << t;

      BOOST_CHECK_EQUAL(out.str(),
                        "(o=[0.176777, 0.306186, 0.176777, 0.918559], p=[5.5, 7.8, 2.1])");
    }

    BOOST_AUTO_TEST_CASE(Transform_From_String)
    {
      Transform t("(o=[0.176777, 0.306186, 0.176777, 0.918559], p=[5.5, 7.8, 2.1])");

      std::stringstream out;
      out << t;

      BOOST_CHECK_EQUAL(out.str(),
                        "(o=[0.176777, 0.306186, 0.176777, 0.918559], p=[5.5, 7.8, 2.1])");
    }

    BOOST_AUTO_TEST_CASE(Transform_From_Boost_Args_Orientation_Only)
    {
      po::variables_map args;
      args.insert(std::make_pair(
          "orientation", po::variable_value(boost::any(std::string("[0, 0, 1] -90.5")), false)));

      Transform t(args);

      Eigen::Quaternionf expectedQuat;
      expectedQuat = Eigen::AngleAxisf(-90.5f * DEG_TO_RAD, Eigen::Vector3f::UnitZ());
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.x(), t.orientation().x());
      BOOST_CHECK_EQUAL(expectedQuat.y(), t.orientation().y());
      BOOST_CHECK_EQUAL(expectedQuat.z(), t.orientation().z());

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

      Eigen::Quaternionf expectedQuat = Eigen::Quaternionf::Identity();
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.x(), t.orientation().x());
      BOOST_CHECK_EQUAL(expectedQuat.y(), t.orientation().y());
      BOOST_CHECK_EQUAL(expectedQuat.z(), t.orientation().z());

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

      Eigen::Quaternionf expectedQuat;
      expectedQuat = Eigen::AngleAxisf(90.0f * DEG_TO_RAD, Eigen::Vector3f::UnitZ());
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.x(), t.orientation().x());
      BOOST_CHECK_EQUAL(expectedQuat.y(), t.orientation().y());
      BOOST_CHECK_EQUAL(expectedQuat.z(), t.orientation().z());

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

      Eigen::Quaternionf expectedQuat = Eigen::Quaternionf::Identity();
      BOOST_CHECK_EQUAL(expectedQuat.w(), t.orientation().w());
      BOOST_CHECK_EQUAL(expectedQuat.x(), t.orientation().x());
      BOOST_CHECK_EQUAL(expectedQuat.y(), t.orientation().y());
      BOOST_CHECK_EQUAL(expectedQuat.z(), t.orientation().z());

      BOOST_CHECK_EQUAL(t.position().x(), 0.0f);
      BOOST_CHECK_EQUAL(t.position().y(), 0.0f);
      BOOST_CHECK_EQUAL(t.position().z(), 0.0f);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}
