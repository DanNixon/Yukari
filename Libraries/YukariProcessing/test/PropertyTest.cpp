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
      BOOST_CHECK(p.isValid());
    }

    BOOST_AUTO_TEST_CASE(Property_Init_Resize)
    {
      Property p;
      BOOST_CHECK_EQUAL(p.size(), 1);
      p.resize(10);
      BOOST_CHECK_EQUAL(p.size(), 10);
      p.resize(5);
      BOOST_CHECK_EQUAL(p.size(), 5);
    }

    BOOST_AUTO_TEST_CASE(Property_Init_Multiple_Values)
    {
      Property p({21, 24, 29});

      std::stringstream str;
      str << p;

      BOOST_CHECK_EQUAL(p.size(), 3);
      BOOST_CHECK_EQUAL(str.str(), "Property[size=3]");
      BOOST_CHECK(p.isValid());

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
        if (values.size() != 3)
          return "Must contain three values";
        if (boost::any_cast<int>(values[1]) != 15)
          return "Second element must equal 15";
        return "";
      };

      Property p1({21});
      Property p2({21, 24, 29});
      Property p3({21, 15, 29});

      p1.setValidator(validator);
      p2.setValidator(validator);
      p3.setValidator(validator);

      BOOST_CHECK_EQUAL(p1.validate(), "Must contain three values");
      BOOST_CHECK(!p1.isValid());
      BOOST_CHECK_EQUAL(p2.validate(), "Second element must equal 15");
      BOOST_CHECK(!p2.isValid());
      BOOST_CHECK_EQUAL(p3.validate(), "");
      BOOST_CHECK(p3.isValid());
    }

    BOOST_AUTO_TEST_CASE(Property_Validation_Remove)
    {
      auto validator = [](const Property::ValueStorageType &) { return "nope"; };

      Property p;

      p.setValidator(validator);
      BOOST_CHECK(!p.isValid());

      p.removeValidator();
      BOOST_CHECK(p.isValid());
    }

    BOOST_AUTO_TEST_CASE(Property_Set_Values)
    {
      Property p(5);

      p[0] = 6;
      p[1] = 14;
      p[2] = std::string("test");
      p[3] = 99;
      p[4] = 107;

      BOOST_CHECK_EQUAL(p.value<int>(0), 6);
      BOOST_CHECK_EQUAL(p.value<int>(1), 14);
      BOOST_CHECK_EQUAL(p.value<std::string>(2), "test");
      BOOST_CHECK_EQUAL(p.value<int>(3), 99);
      BOOST_CHECK_EQUAL(p.value<int>(4), 107);
    }

    BOOST_AUTO_TEST_CASE(Property_Copy_Empty_No_Validator)
    {
      Property p1(5);
      BOOST_CHECK_EQUAL(p1.size(), 5);
      BOOST_CHECK(p1.isValid());

      Property p2(p1);
      BOOST_CHECK_EQUAL(p2.size(), 5);
      BOOST_CHECK(p2.isValid());
    }

    BOOST_AUTO_TEST_CASE(Property_Copy_Values_No_Validator)
    {
      Property p1({2, 4, 6, 8, 10});
      BOOST_CHECK_EQUAL(p1.size(), 5);
      BOOST_CHECK(p1.isValid());

      BOOST_CHECK_EQUAL(p1.value<int>(0), 2);
      BOOST_CHECK_EQUAL(p1.value<int>(1), 4);
      BOOST_CHECK_EQUAL(p1.value<int>(2), 6);
      BOOST_CHECK_EQUAL(p1.value<int>(3), 8);
      BOOST_CHECK_EQUAL(p1.value<int>(4), 10);

      Property p2(p1);
      BOOST_CHECK_EQUAL(p2.size(), 5);
      BOOST_CHECK(p2.isValid());

      BOOST_CHECK_EQUAL(p2.value<int>(0), 2);
      BOOST_CHECK_EQUAL(p2.value<int>(1), 4);
      BOOST_CHECK_EQUAL(p2.value<int>(2), 6);
      BOOST_CHECK_EQUAL(p2.value<int>(3), 8);
      BOOST_CHECK_EQUAL(p2.value<int>(4), 10);
    }

    BOOST_AUTO_TEST_CASE(Property_Copy_Values_Validator)
    {
      Property p1({2, 4, 6, 8, 10});
      p1.setValidator([](const Property::ValueStorageType &values) {
        return ((boost::any_cast<int>(values[2]) == 50) ? "" : "Third element must be 50");
      });
      BOOST_CHECK_EQUAL(p1.size(), 5);
      BOOST_CHECK(!p1.isValid());

      BOOST_CHECK_EQUAL(p1.value<int>(0), 2);
      BOOST_CHECK_EQUAL(p1.value<int>(1), 4);
      BOOST_CHECK_EQUAL(p1.value<int>(2), 6);
      BOOST_CHECK_EQUAL(p1.value<int>(3), 8);
      BOOST_CHECK_EQUAL(p1.value<int>(4), 10);

      Property p2(p1);
      BOOST_CHECK_EQUAL(p2.size(), 5);
      BOOST_CHECK(!p2.isValid());

      BOOST_CHECK_EQUAL(p2.value<int>(0), 2);
      BOOST_CHECK_EQUAL(p2.value<int>(1), 4);
      BOOST_CHECK_EQUAL(p2.value<int>(2), 6);
      BOOST_CHECK_EQUAL(p2.value<int>(3), 8);
      BOOST_CHECK_EQUAL(p2.value<int>(4), 10);

      p2[2] = 50;
      BOOST_CHECK(p2.isValid());

      BOOST_CHECK_EQUAL(p2.value<int>(2), 50);
    }

    BOOST_AUTO_TEST_CASE(Property_Copy_Modify_Values)
    {
      /* Create a property with some values */
      Property p1(5);
      p1[0] = 10;
      p1[1] = 20;
      p1[2] = 30;
      p1[3] = 40;
      p1[4] = 50;
      BOOST_CHECK_EQUAL(p1.value<int>(0), 10);
      BOOST_CHECK_EQUAL(p1.value<int>(1), 20);
      BOOST_CHECK_EQUAL(p1.value<int>(2), 30);
      BOOST_CHECK_EQUAL(p1.value<int>(3), 40);
      BOOST_CHECK_EQUAL(p1.value<int>(4), 50);

      /* Copy the property */
      Property p2(p1);
      BOOST_CHECK_EQUAL(p2.value<int>(0), 10);
      BOOST_CHECK_EQUAL(p2.value<int>(1), 20);
      BOOST_CHECK_EQUAL(p2.value<int>(2), 30);
      BOOST_CHECK_EQUAL(p2.value<int>(3), 40);
      BOOST_CHECK_EQUAL(p2.value<int>(4), 50);

      /* Change some values in the copy */
      p2[0] = 11;
      p2[1] = 22;
      p2[2] = 33;
      p2[3] = 44;
      p2[4] = 55;
      BOOST_CHECK_EQUAL(p2.value<int>(0), 11);
      BOOST_CHECK_EQUAL(p2.value<int>(1), 22);
      BOOST_CHECK_EQUAL(p2.value<int>(2), 33);
      BOOST_CHECK_EQUAL(p2.value<int>(3), 44);
      BOOST_CHECK_EQUAL(p2.value<int>(4), 55);

      /* Check values in p1 are unchanged */
      BOOST_CHECK_EQUAL(p1.value<int>(0), 10);
      BOOST_CHECK_EQUAL(p1.value<int>(1), 20);
      BOOST_CHECK_EQUAL(p1.value<int>(2), 30);
      BOOST_CHECK_EQUAL(p1.value<int>(3), 40);
      BOOST_CHECK_EQUAL(p1.value<int>(4), 50);
    }

    BOOST_AUTO_TEST_CASE(Property_Copy_Modify_Pointers_Assign)
    {
      typedef std::shared_ptr<int> IntType;

      /* Create a property with some values */
      Property p1(5);
      p1[0] = std::make_shared<int>(10);
      p1[1] = std::make_shared<int>(20);
      p1[2] = std::make_shared<int>(30);
      p1[3] = std::make_shared<int>(40);
      p1[4] = std::make_shared<int>(50);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(0), 10);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(1), 20);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(2), 30);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(3), 40);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(4), 50);

      /* Copy the property */
      Property p2(p1);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(0), 10);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(1), 20);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(2), 30);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(3), 40);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(4), 50);

      /* Change some values in the copy */
      p2[0] = std::make_shared<int>(11);
      p2[1] = std::make_shared<int>(22);
      p2[2] = std::make_shared<int>(33);
      p2[3] = std::make_shared<int>(44);
      p2[4] = std::make_shared<int>(55);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(0), 11);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(1), 22);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(2), 33);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(3), 44);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(4), 55);

      /* Check values in p1 are unchanged */
      BOOST_CHECK_EQUAL(*p1.value<IntType>(0), 10);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(1), 20);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(2), 30);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(3), 40);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(4), 50);
    }

    BOOST_AUTO_TEST_CASE(Property_Copy_Modify_Pointers_Modify)
    {
      typedef std::shared_ptr<int> IntType;

      /* Create a property with some values */
      Property p1(5);
      p1[0] = std::make_shared<int>(10);
      p1[1] = std::make_shared<int>(20);
      p1[2] = std::make_shared<int>(30);
      p1[3] = std::make_shared<int>(40);
      p1[4] = std::make_shared<int>(50);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(0), 10);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(1), 20);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(2), 30);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(3), 40);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(4), 50);

      /* Copy the property */
      Property p2(p1);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(0), 10);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(1), 20);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(2), 30);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(3), 40);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(4), 50);

      /* Change some values in the copy */
      *p2.value<IntType>(0) = 11;
      *p2.value<IntType>(1) = 22;
      *p2.value<IntType>(2) = 33;
      *p2.value<IntType>(3) = 44;
      *p2.value<IntType>(4) = 55;
      BOOST_CHECK_EQUAL(*p2.value<IntType>(0), 11);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(1), 22);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(2), 33);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(3), 44);
      BOOST_CHECK_EQUAL(*p2.value<IntType>(4), 55);

      /* Values in p1 are changed */
      BOOST_CHECK_EQUAL(*p1.value<IntType>(0), 11);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(1), 22);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(2), 33);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(3), 44);
      BOOST_CHECK_EQUAL(*p1.value<IntType>(4), 55);
    }

    BOOST_AUTO_TEST_CASE(Property_Set_Vector_1)
    {
      Property p(10);
      BOOST_CHECK_EQUAL(p.size(), 10);

      std::vector<boost::any> values = {3, 8, 7, 6, 9};

      p.set(values);
      BOOST_CHECK_EQUAL(p.size(), values.size());

      BOOST_CHECK_EQUAL(p.value<int>(0), 3);
      BOOST_CHECK_EQUAL(p.value<int>(1), 8);
      BOOST_CHECK_EQUAL(p.value<int>(2), 7);
      BOOST_CHECK_EQUAL(p.value<int>(3), 6);
      BOOST_CHECK_EQUAL(p.value<int>(4), 9);
    }

    BOOST_AUTO_TEST_CASE(Property_Set_Vector_2)
    {
      Property p({6, 3, 2, 1, 7, 8, 6, 3, 7, 5});
      BOOST_CHECK_EQUAL(p.size(), 10);

      std::vector<boost::any> values = {3, 8, 7, 6, 9};

      p.set(values);
      BOOST_CHECK_EQUAL(p.size(), values.size());

      BOOST_CHECK_EQUAL(p.value<int>(0), 3);
      BOOST_CHECK_EQUAL(p.value<int>(1), 8);
      BOOST_CHECK_EQUAL(p.value<int>(2), 7);
      BOOST_CHECK_EQUAL(p.value<int>(3), 6);
      BOOST_CHECK_EQUAL(p.value<int>(4), 9);
    }
  }
}
}

#endif
