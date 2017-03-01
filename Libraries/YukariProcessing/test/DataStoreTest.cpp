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

    BOOST_AUTO_TEST_CASE(DataStore_Add_Properties)
    {
      Property_sptr p1 = std::make_shared<Property>();
      Property_sptr p2 = std::make_shared<Property>();
      Property_sptr p3 = std::make_shared<Property>();
      Property_sptr p4 = std::make_shared<Property>();

      DataStore ds;

      ds["p1"] = p1;
      ds["p2"] = p2;
      ds["p3"] = p3;
      ds["p4"] = p4;

      std::stringstream str;
      str << ds;

      BOOST_CHECK_EQUAL(ds.size(), 4);
      BOOST_CHECK_EQUAL(str.str(), "DataStore[size=4]");
    }

    BOOST_AUTO_TEST_CASE(DataStore_Pretty_Print)
    {
      Property_sptr p1 = std::make_shared<Property>(0);
      Property_sptr p2 = std::make_shared<Property>(7);
      Property_sptr p3 = std::make_shared<Property>(12);
      Property_sptr p4 = std::make_shared<Property>(10);

      DataStore ds;

      ds["p1"] = p1;
      ds["p2"] = p2;
      ds["p3"] = p3;
      ds["p4"] = p4;

      std::stringstream str;
      ds.prettyPrint(str);

      const std::string expected("p1 (0)\np2 (7)\np3 (12)\np4 (10)\n");

      BOOST_CHECK_EQUAL(str.str(), expected);
    }

    BOOST_AUTO_TEST_CASE(DataStore_Find_By_Regex_Whole_Name)
    {
      Property_sptr p1 = std::make_shared<Property>();
      Property_sptr p2 = std::make_shared<Property>();
      Property_sptr p3 = std::make_shared<Property>();
      Property_sptr p4 = std::make_shared<Property>();

      DataStore ds;

      ds["p1"] = p1;
      ds["p2"] = p2;
      ds["p3"] = p3;
      ds["p4"] = p4;

      BOOST_CHECK(ds.findByRegex("p1")[0].second == p1);
      BOOST_CHECK(ds.findByRegex("p2")[0].second == p2);
      BOOST_CHECK(ds.findByRegex("p3")[0].second == p3);
      BOOST_CHECK(ds.findByRegex("p4")[0].second == p4);
    }

    BOOST_AUTO_TEST_CASE(DataStore_Find_By_Regex_Partial)
    {
      Property_sptr p1 = std::make_shared<Property>();
      Property_sptr p2 = std::make_shared<Property>();
      Property_sptr p3 = std::make_shared<Property>();
      Property_sptr p4 = std::make_shared<Property>();

      DataStore ds;

      ds["input_point_clouds"] = p1;
      ds["output_point_clouds"] = p2;
      ds["filtered_point_clouds"] = p3;
      ds["output_meshes"] = p4;

      {
        boost::regex re(".*point_clouds");

        DataStore::ItemList results = ds.findByRegex(re);
        BOOST_CHECK_EQUAL(results.size(), 3);

        BOOST_CHECK(std::find_if(results.begin(), results.end(), [&p1](DataStore::Item i) {
                      return i.second == p1;
                    }) != results.end());

        BOOST_CHECK(std::find_if(results.begin(), results.end(), [&p2](DataStore::Item i) {
                      return i.second == p2;
                    }) != results.end());

        BOOST_CHECK(std::find_if(results.begin(), results.end(), [&p3](DataStore::Item i) {
                      return i.second == p3;
                    }) != results.end());
      }

      {
        boost::regex re("output.*");

        DataStore::ItemList results = ds.findByRegex(re);
        BOOST_CHECK_EQUAL(results.size(), 2);

        BOOST_CHECK(std::find_if(results.begin(), results.end(), [&p2](DataStore::Item i) {
                      return i.second == p2;
                    }) != results.end());

        BOOST_CHECK(std::find_if(results.begin(), results.end(), [&p4](DataStore::Item i) {
                      return i.second == p4;
                    }) != results.end());
      }
    }
  }
}
}

#endif
