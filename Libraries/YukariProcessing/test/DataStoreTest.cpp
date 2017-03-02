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

    BOOST_AUTO_TEST_CASE(DataStore_Contains)
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

      BOOST_CHECK(ds.contains("p1"));
      BOOST_CHECK(ds.contains("p2"));
      BOOST_CHECK(ds.contains("p3"));
      BOOST_CHECK(ds.contains("p4"));
      BOOST_CHECK(!ds.contains("p5"));
      BOOST_CHECK(!ds.contains("p6"));
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

    BOOST_AUTO_TEST_CASE(DataStore_Pretty_Print_Regex)
    {
      Property_sptr p1 = std::make_shared<Property>(0);
      Property_sptr p2 = std::make_shared<Property>(7);
      Property_sptr p3 = std::make_shared<Property>(12);
      Property_sptr p4 = std::make_shared<Property>(10);

      DataStore ds;

      ds["input_1"] = p1;
      ds["output_1"] = p2;
      ds["input_2"] = p3;
      ds["output_2"] = p4;

      {
        std::stringstream str;
        ds.prettyPrint(str, boost::regex("input_.*"));
        const std::string expected("input_1 (0)\ninput_2 (12)\n");
        BOOST_CHECK_EQUAL(str.str(), expected);
      }

      {
        std::stringstream str;
        ds.prettyPrint(str, boost::regex(".*_2"));
        const std::string expected("input_2 (12)\noutput_2 (10)\n");
        BOOST_CHECK_EQUAL(str.str(), expected);
      }
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

      BOOST_CHECK(ds.findByRegex(boost::regex("p1"))[0].second == p1);
      BOOST_CHECK(ds.findByRegex(boost::regex("p2"))[0].second == p2);
      BOOST_CHECK(ds.findByRegex(boost::regex("p3"))[0].second == p3);
      BOOST_CHECK(ds.findByRegex(boost::regex("p4"))[0].second == p4);
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

    BOOST_AUTO_TEST_CASE(DataStore_Delete_By_Regex_Whole_Name)
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

      {
        auto retVal = ds.deleteByRegex(boost::regex("p2"));
        BOOST_CHECK_EQUAL(retVal.size(), 1);
        BOOST_CHECK_EQUAL(retVal[0], "p2");

        BOOST_CHECK_EQUAL(ds.size(), 3);
      }

      {
        auto retVal = ds.deleteByRegex(boost::regex("p1"));
        BOOST_CHECK_EQUAL(retVal.size(), 1);
        BOOST_CHECK_EQUAL(retVal[0], "p1");

        BOOST_CHECK_EQUAL(ds.size(), 2);
      }

      {
        auto retVal = ds.deleteByRegex(boost::regex("p4"));
        BOOST_CHECK_EQUAL(retVal.size(), 1);
        BOOST_CHECK_EQUAL(retVal[0], "p4");

        BOOST_CHECK_EQUAL(ds.size(), 1);
      }

      {
        auto retVal = ds.deleteByRegex(boost::regex("p5"));
        BOOST_CHECK(retVal.empty());

        BOOST_CHECK_EQUAL(ds.size(), 1);
      }

      {
        auto retVal = ds.deleteByRegex(boost::regex("p3"));
        BOOST_CHECK_EQUAL(retVal.size(), 1);
        BOOST_CHECK_EQUAL(retVal[0], "p3");

        BOOST_CHECK_EQUAL(ds.size(), 0);
      }
    }

    BOOST_AUTO_TEST_CASE(DataStore_Delete_By_Regex_Partial_1)
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

      std::vector<std::string> results = ds.deleteByRegex(boost::regex(".*point_clouds"));
      BOOST_CHECK_EQUAL(results.size(), 3);

      BOOST_CHECK(std::find_if(results.begin(), results.end(), [](const std::string &s) {
                    return s == "input_point_clouds";
                  }) != results.end());

      BOOST_CHECK(std::find_if(results.begin(), results.end(), [](const std::string &s) {
                    return s == "output_point_clouds";
                  }) != results.end());

      BOOST_CHECK(std::find_if(results.begin(), results.end(), [](const std::string &s) {
                    return s == "filtered_point_clouds";
                  }) != results.end());

      BOOST_CHECK_EQUAL(ds.size(), 1);
    }

    BOOST_AUTO_TEST_CASE(DataStore_Delete_By_Regex_Partial_2)
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

      std::vector<std::string> results = ds.deleteByRegex(boost::regex("output.*"));
      BOOST_CHECK_EQUAL(results.size(), 2);

      BOOST_CHECK(std::find_if(results.begin(), results.end(), [](const std::string &s) {
                    return s == "output_point_clouds";
                  }) != results.end());

      BOOST_CHECK(std::find_if(results.begin(), results.end(), [](const std::string &s) {
                    return s == "output_meshes";
                  }) != results.end());

      BOOST_CHECK_EQUAL(ds.size(), 2);
    }
  }
}
}

#endif
