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
    }
  }
}
}

#endif
