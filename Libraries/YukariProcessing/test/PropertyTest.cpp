/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "PropertyTest"

#include <boost/test/unit_test.hpp>

#include <strstream>

#include <YukariProcessing/Property.h>

namespace Yukari
{
namespace Processing
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(Property_Init_Empty)
    {
      Property p;

      std::stringstream str;
      str << p;

      BOOST_CHECK_EQUAL(p.size(), 1);
      BOOST_CHECK_EQUAL(str.str(), "Property[size=1]");
      BOOST_CHECK(p.validate());
    }

    BOOST_AUTO_TEST_CASE(Property_Init_Multiple_Values)
    {
      Property p({21, 24, 29});

      std::stringstream str;
      str << p;

      BOOST_CHECK_EQUAL(p.size(), 3);
      BOOST_CHECK_EQUAL(str.str(), "Property[size=3]");
      BOOST_CHECK(p.validate());

      BOOST_CHECK(p.isA<int>());
      BOOST_CHECK(p.isA<int>(0));

      BOOST_CHECK_EQUAL(p.value<int>(), 21);
      BOOST_CHECK_EQUAL(p.value<int>(0), 21);
      BOOST_CHECK_EQUAL(p.value<int>(1), 24);
      BOOST_CHECK_EQUAL(p.value<int>(2), 29);
    }

    BOOST_AUTO_TEST_CASE(Property_Validation)
    {
      /* 3 elements and second element must equal 15 */
      auto validator = [](const Property::ValueStorageType &values) {
        return values.size() == 3 && boost::any_cast<int>(values[1]) == 15;
      };

      Property p1({21});
      Property p2({21, 24, 29});
      Property p3({21, 15, 29});

      p1.setValidator(validator);
      p2.setValidator(validator);
      p3.setValidator(validator);

      BOOST_CHECK(!p1.validate());
      BOOST_CHECK(!p2.validate());
      BOOST_CHECK(p3.validate());
    }

    BOOST_AUTO_TEST_CASE(Property_Validation_Remove)
    {
      auto validator = [](const Property::ValueStorageType &) {
        return false;
      };

      Property p;

      p.setValidator(validator);
      BOOST_CHECK(!p.validate());

      p.removeValidator();
      BOOST_CHECK(p.validate());
    }

    BOOST_AUTO_TEST_CASE(Property_Set_Values)
    {
      Property p(5);

      p[0] = 6;
      p[1] = 14;
      p[2] = 756;
      p[3] = 99;
      p[4] = 107;

      BOOST_CHECK_EQUAL(p.value<int>(0), 6);
      BOOST_CHECK_EQUAL(p.value<int>(1), 14);
      BOOST_CHECK_EQUAL(p.value<int>(2), 756);
      BOOST_CHECK_EQUAL(p.value<int>(3), 99);
      BOOST_CHECK_EQUAL(p.value<int>(4), 107);
    }
  }
}
}

#endif
