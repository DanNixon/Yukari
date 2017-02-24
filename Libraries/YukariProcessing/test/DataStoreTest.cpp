/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "DataStoreTest"

#include <boost/test/unit_test.hpp>

#include <sstream>

#include <YukariProcessing/DataStore.h>

namespace Yukari
{
namespace Processing
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(DataStore_Init_Empty)
    {
      DataStore ds;

      std::stringstream str;
      str << ds;

      BOOST_CHECK_EQUAL(ds.size(), 0);
      BOOST_CHECK_EQUAL(str.str(), "DataStore[size=0]");
    }
  }
}
}

#endif
