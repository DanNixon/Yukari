#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "StringValueConversionTest"

#include <boost/test/unit_test.hpp>

#include <YukariCommon/StringValueConversion.h>

namespace Yukari
{
namespace Common
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(StringValueConversion_Valid_Conversions)
    {
      /* Int */
      {
        auto v = StringValueConversion::Convert("int", "0");
        BOOST_CHECK_EQUAL(boost::any_cast<int>(v), 0);
      }

      {
        auto v = StringValueConversion::Convert("int", "20");
        BOOST_CHECK_EQUAL(boost::any_cast<int>(v), 20);
      }

      {
        auto v = StringValueConversion::Convert("int", "-5");
        BOOST_CHECK_EQUAL(boost::any_cast<int>(v), -5);
      }

      /* Float */
      {
        auto v = StringValueConversion::Convert("float", "0");
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 0.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", "0.0");
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 0.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", "1");
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 1.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", "1.0");
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 1.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", "0.5");
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 0.5f);
      }

      /* String */
      {
        auto v = StringValueConversion::Convert("string", "");
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "");
      }

      {
        auto v = StringValueConversion::Convert("string", "-5");
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "-5");
      }

      {
        auto v = StringValueConversion::Convert("string", "hello world");
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }
    }

    BOOST_AUTO_TEST_CASE(StringValueConversion_Spaces_In_Names)
    {
      {
        auto v = StringValueConversion::Convert(" string ", "hello world");
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }
    }

    BOOST_AUTO_TEST_CASE(StringValueConversion_Different_Names)
    {
      {
        auto v = StringValueConversion::Convert("STRING", "hello world");
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }

      {
        auto v = StringValueConversion::Convert("StRiNg", "hello world");
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }
    }

    BOOST_AUTO_TEST_CASE(StringValueConversion_Valid_Conversions_Multiple)
    {
      std::vector<std::string> values({"2", "4", "0", "-1", "17", "-100"});

      std::vector<boost::any> result = StringValueConversion::Convert("int", values);
      BOOST_CHECK_EQUAL(result.size(), values.size());

      BOOST_CHECK_EQUAL(boost::any_cast<int>(result[0]), 2);
      BOOST_CHECK_EQUAL(boost::any_cast<int>(result[1]), 4);
      BOOST_CHECK_EQUAL(boost::any_cast<int>(result[2]), 0);
      BOOST_CHECK_EQUAL(boost::any_cast<int>(result[3]), -1);
      BOOST_CHECK_EQUAL(boost::any_cast<int>(result[4]), 17);
      BOOST_CHECK_EQUAL(boost::any_cast<int>(result[5]), -100);
    }
  };
}
}

#endif
