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

		BOOST_CHECK_EQUAL(p.value<int>(), 21);
		BOOST_CHECK_EQUAL(p.value<int>(0), 21);
		BOOST_CHECK_EQUAL(p.value<int>(1), 24);
		BOOST_CHECK_EQUAL(p.value<int>(2), 29);
	}

	// TODO
  }
}
}

#endif
