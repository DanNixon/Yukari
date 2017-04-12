/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariCommon/MapHelpers.h>

namespace Yukari
{
namespace Common
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(MapHelpersTest)

    BOOST_AUTO_TEST_CASE(Get_InMap)
    {
      std::map<std::string, std::string> m = {{"a", "b"}, {"c", "d"}};
      std::string result = MapHelpers::Get<std::string, std::string>(m, "a");
      BOOST_CHECK_EQUAL(result, std::string("b"));
    }

    BOOST_AUTO_TEST_CASE(Get_NotInMap)
    {
      std::map<std::string, std::string> m = {{"a", "b"}, {"c", "d"}};
      std::string result = MapHelpers::Get<std::string, std::string>(m, "e", "j");
      BOOST_CHECK_EQUAL(result, std::string("j"));
    }

    BOOST_AUTO_TEST_CASE(Get_NotInMapTypeDefault)
    {
      std::map<std::string, std::string> m = {{"a", "b"}, {"c", "d"}};
      std::string result = MapHelpers::Get<std::string, std::string>(m, "e");
      BOOST_CHECK_EQUAL(result, std::string(""));
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
