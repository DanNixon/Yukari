/** @file */

#include <boost/test/unit_test.hpp>

#include <YukariCommon/StringParsers.h>

namespace Yukari
{
namespace Common
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(StringParsersTest)

    BOOST_AUTO_TEST_CASE(CleanString_1)
    {
      std::string s = "  test1234   \t";

      StringParsers::CleanString(s);
      BOOST_CHECK_EQUAL(s, std::string("test1234"));
    }

    BOOST_AUTO_TEST_CASE(CleanString_2)
    {
      std::string s = "T EST4 2781";

      StringParsers::CleanString(s);
      BOOST_CHECK_EQUAL(s, std::string("t est4 2781"));
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_EmptyString)
    {
      std::string in = "";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(!StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string(""));

      BOOST_CHECK_EQUAL(params.size(), 0);
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_CommandOnly)
    {
      std::string in = "TestThing";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("TestThing"));

      BOOST_CHECK_EQUAL(params.size(), 0);
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_NoParameters)
    {
      std::string in = "TestThing()";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("TestThing"));

      BOOST_CHECK_EQUAL(params.size(), 0);
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_SingleParameter)
    {
      std::string in = "TestThing(a=abc123)";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("TestThing"));

      BOOST_CHECK_EQUAL(params.size(), 1);
      BOOST_CHECK_EQUAL(params["a"], std::string("abc123"));
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_SingleParameter_Spaces)
    {
      std::string in = " TestThing (a=abc123) ";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("TestThing"));

      BOOST_CHECK_EQUAL(params.size(), 1);
      BOOST_CHECK_EQUAL(params["a"], std::string("abc123"));
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_TwoParameters)
    {
      std::string in = "TestThing(a=abc123, b=def456)";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("TestThing"));

      BOOST_CHECK_EQUAL(params.size(), 2);
      BOOST_CHECK_EQUAL(params["a"], std::string("abc123"));
      BOOST_CHECK_EQUAL(params["b"], std::string("def456"));
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_Slashes)
    {
      std::string in = "Teensy(port=/dev/ttyACM0, this=one\\the other)";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("Teensy"));

      BOOST_CHECK_EQUAL(params.size(), 2);
      BOOST_CHECK_EQUAL(params["port"], std::string("/dev/ttyACM0"));
      BOOST_CHECK_EQUAL(params["this"], std::string("one\\the other"));
    }

    BOOST_AUTO_TEST_CASE(ParseCommand_Regex)
    {
      std::string in = "file(pattern=.*\\_imu\\.txt, delay=1000)";

      std::string cmd;
      std::map<std::string, std::string> params;

      BOOST_CHECK(StringParsers::ParseCommand(in, cmd, params));

      BOOST_CHECK_EQUAL(cmd, std::string("file"));

      BOOST_CHECK_EQUAL(params.size(), 2);
      BOOST_CHECK_EQUAL(params["pattern"], std::string(".*\\_imu\\.txt"));
      BOOST_CHECK_EQUAL(params["delay"], std::string("1000"));
    }

    BOOST_AUTO_TEST_CASE(ParseQuaternion)
    {
      std::string in = "[5, -3, 9.9, 1.0]";

      Eigen::Quaternionf q = StringParsers::ParseQuaternion(in);

      BOOST_CHECK_EQUAL(q.w(), 1.0f);
      BOOST_CHECK_EQUAL(q.x(), 5.0f);
      BOOST_CHECK_EQUAL(q.y(), -3.0f);
      BOOST_CHECK_EQUAL(q.z(), 9.9f);
    }

    BOOST_AUTO_TEST_CASE(ParseVector)
    {
      std::string in = "[5, -3, 9.9]";

      Eigen::Vector3f v = StringParsers::ParseVector(in);

      BOOST_CHECK_EQUAL(v.x(), 5.0f);
      BOOST_CHECK_EQUAL(v.y(), -3.0f);
      BOOST_CHECK_EQUAL(v.z(), 9.9f);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}
