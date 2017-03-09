#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariCommon/StringValueConversion.h>

namespace Yukari
{
namespace Common
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(StringValueConversionTest)

    BOOST_AUTO_TEST_CASE(StringValueConversion_Valid_Conversions)
    {
      /* Int */
      {
        auto v = StringValueConversion::Convert("int", std::string("0"));
        BOOST_CHECK_EQUAL(boost::any_cast<int>(v), 0);
      }

      {
        auto v = StringValueConversion::Convert("int", std::string("20"));
        BOOST_CHECK_EQUAL(boost::any_cast<int>(v), 20);
      }

      {
        auto v = StringValueConversion::Convert("int", std::string("-5"));
        BOOST_CHECK_EQUAL(boost::any_cast<int>(v), -5);
      }

      /* Float */
      {
        auto v = StringValueConversion::Convert("float", std::string("0"));
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 0.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", std::string("0.0"));
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 0.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", std::string("1"));
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 1.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", std::string("1.0"));
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 1.0f);
      }

      {
        auto v = StringValueConversion::Convert("float", std::string("0.5"));
        BOOST_CHECK_EQUAL(boost::any_cast<float>(v), 0.5f);
      }

      /* String */
      {
        auto v = StringValueConversion::Convert("string", std::string(""));
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "");
      }

      {
        auto v = StringValueConversion::Convert("string", std::string("-5"));
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "-5");
      }

      {
        auto v = StringValueConversion::Convert("string", std::string("hello world"));
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }
    }

    BOOST_AUTO_TEST_CASE(StringValueConversion_Spaces_In_Names)
    {
      {
        auto v = StringValueConversion::Convert(" string ", std::string("hello world"));
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }
    }

    BOOST_AUTO_TEST_CASE(StringValueConversion_Different_Names)
    {
      {
        auto v = StringValueConversion::Convert("STRING", std::string("hello world"));
        BOOST_CHECK_EQUAL(boost::any_cast<std::string>(v), "hello world");
      }

      {
        auto v = StringValueConversion::Convert("StRiNg", std::string("hello world"));
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

    BOOST_AUTO_TEST_CASE(StringValueConversion_Valid_Conversions_To_String)
    {
      /* Int */
      {
        boost::any a(0);
        std::string v = StringValueConversion::Convert("int", a);
        BOOST_CHECK_EQUAL(v, "0");
      }

      {
        boost::any a(20);
        std::string v = StringValueConversion::Convert("int", a);
        BOOST_CHECK_EQUAL(v, "20");
      }

      {
        boost::any a(-5);
        std::string v = StringValueConversion::Convert("int", a);
        BOOST_CHECK_EQUAL(v, "-5");
      }

      /* Float */
      {
        boost::any a(0.0f);
        std::string v = StringValueConversion::Convert("float", a);
        BOOST_CHECK_EQUAL(v, "0.000000");
      }

      {
        boost::any a(1.0f);
        std::string v = StringValueConversion::Convert("float", a);
        BOOST_CHECK_EQUAL(v, "1.000000");
      }

      {
        boost::any a(0.5f);
        std::string v = StringValueConversion::Convert("float", a);
        BOOST_CHECK_EQUAL(v, "0.500000");
      }

      /* String */
      {
        boost::any a(std::string(""));
        std::string v = StringValueConversion::Convert("string", a);
        BOOST_CHECK_EQUAL(v, "");
      }

      {
        boost::any a(std::string("-5"));
        std::string v = StringValueConversion::Convert("string", a);
        BOOST_CHECK_EQUAL(v, "-5");
      }

      {
        boost::any a(std::string("hello world"));
        std::string v = StringValueConversion::Convert("string", a);
        BOOST_CHECK_EQUAL(v, "hello world");
      }
    }

    BOOST_AUTO_TEST_SUITE_END()
  };
}
}

#endif
