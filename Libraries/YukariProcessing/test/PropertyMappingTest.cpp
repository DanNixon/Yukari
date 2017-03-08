/** @file */

#ifndef DOXYGEN_SKIP

#include <boost/test/unit_test.hpp>

#include <YukariProcessing/PropertyMapping.h>

namespace Yukari
{
namespace Processing
{
  namespace Test
  {
    class MockAlgorithm : public IAlgorithm
    {
    protected:
      virtual void doExecute() override
      {
        Property_sptr a = getProperty(Processing::INPUT, "a");
        Property_sptr b = getProperty(Processing::INPUT, "b");
        size_t num = a->size();
        Property_sptr z = std::make_shared<Property>(num);

        for (size_t i = 0; i < num; i++)
          (*z)[i] = a->value<int>(i) + b->value<int>(i);

        setProperty(Processing::OUTPUT, "z", z);
      }
    };

    BOOST_AUTO_TEST_SUITE(PropertyMappingTest)

    BOOST_AUTO_TEST_CASE(PropertyMappingItem_Input_Stream_Out)
    {
      PropertyMappingItem i;
      i.direction = INPUT;
      i.propertyName = "prop";
      i.dataName = "data";

      std::stringstream s;
      s << i;

      BOOST_CHECK_EQUAL(s.str(), "Mapping(data -> [prop])");
    }

    BOOST_AUTO_TEST_CASE(PropertyMappingItem_Output_Stream_Out)
    {
      PropertyMappingItem i;
      i.direction = OUTPUT;
      i.propertyName = "prop";
      i.dataName = "data";

      std::stringstream s;
      s << i;

      BOOST_CHECK_EQUAL(s.str(), "Mapping([prop] -> data)");
    }

    BOOST_AUTO_TEST_CASE(PropertyMapping_Empty)
    {
      PropertyMapping p;
      BOOST_CHECK_EQUAL(p.size(), 0);

      std::stringstream s;
      s << p;

      BOOST_CHECK_EQUAL(s.str(), "PropertyMapping[Input[], Output[]]");
    }

    BOOST_AUTO_TEST_CASE(PropertyMapping_Parse_Strings)
    {
      PropertyMapping p;

      BOOST_CHECK(p.parseString("data_a -> [property_1]"));
      BOOST_CHECK(p.parseString("[property_2] ->data_b"));
      BOOST_CHECK(p.parseString("[property_3]->data_c"));
      BOOST_CHECK(p.parseString("data_d-> [property_4]"));

      BOOST_CHECK_EQUAL(p.size(), 4);

      BOOST_CHECK_EQUAL(p[0].direction, INPUT);
      BOOST_CHECK_EQUAL(p[0].dataName, "data_a");
      BOOST_CHECK_EQUAL(p[0].propertyName, "property_1");

      BOOST_CHECK_EQUAL(p[1].direction, OUTPUT);
      BOOST_CHECK_EQUAL(p[1].dataName, "data_b");
      BOOST_CHECK_EQUAL(p[1].propertyName, "property_2");

      BOOST_CHECK_EQUAL(p[2].direction, OUTPUT);
      BOOST_CHECK_EQUAL(p[2].dataName, "data_c");
      BOOST_CHECK_EQUAL(p[2].propertyName, "property_3");

      BOOST_CHECK_EQUAL(p[3].direction, INPUT);
      BOOST_CHECK_EQUAL(p[3].dataName, "data_d");
      BOOST_CHECK_EQUAL(p[3].propertyName, "property_4");
    }

    BOOST_AUTO_TEST_CASE(PropertyMapping_Parse_Strings_Errors)
    {
      PropertyMapping p;

      BOOST_CHECK(!p.parseString("->[prop1]"));
      BOOST_CHECK(!p.parseString("data- >[prop2]"));
      BOOST_CHECK(!p.parseString("data-[prop3]"));
      BOOST_CHECK(!p.parseString("data->prop"));
      BOOST_CHECK(!p.parseString("[data]->[prop]"));

      BOOST_CHECK_EQUAL(p.size(), 0);
    }

    BOOST_AUTO_TEST_CASE(PropertyMapping_Parse_Strings_Multiple)
    {
      // clang-format off
      std::vector<std::string> pairs = {
          "data_a -> [property_1]",
		  "data- >[prop2]",
		  "data-[prop3]",
		  "[property_2] ->data_b",
          "[property_3]->data_c",
		  "data->prop",
		  "->[prop1]",
		  "data_d-> [property_4]",
          "[data]->[prop]"};
      // clang-format on

      PropertyMapping p;

      BOOST_CHECK_EQUAL(p.parseString(pairs.begin(), pairs.end()), 4);
      BOOST_CHECK_EQUAL(p.size(), 4);

      BOOST_CHECK_EQUAL(p[0].direction, INPUT);
      BOOST_CHECK_EQUAL(p[0].dataName, "data_a");
      BOOST_CHECK_EQUAL(p[0].propertyName, "property_1");

      BOOST_CHECK_EQUAL(p[1].direction, OUTPUT);
      BOOST_CHECK_EQUAL(p[1].dataName, "data_b");
      BOOST_CHECK_EQUAL(p[1].propertyName, "property_2");

      BOOST_CHECK_EQUAL(p[2].direction, OUTPUT);
      BOOST_CHECK_EQUAL(p[2].dataName, "data_c");
      BOOST_CHECK_EQUAL(p[2].propertyName, "property_3");

      BOOST_CHECK_EQUAL(p[3].direction, INPUT);
      BOOST_CHECK_EQUAL(p[3].dataName, "data_d");
      BOOST_CHECK_EQUAL(p[3].propertyName, "property_4");
    }

    BOOST_AUTO_TEST_CASE(PropertyMapping_Set_Properties)
    {
      /* Test data */
      DataStore_sptr ds = std::make_shared<DataStore>();
      (*ds)["a_data"] = Property_sptr(new Property({1, 2, 3, 4, 5}));
      (*ds)["b_data"] = Property_sptr(new Property({10, 9, 8, 7, 6}));

      /* Test mapping */
      PropertyMapping pm;
      pm.push_back({INPUT, "a", "a_data"});
      pm.push_back({INPUT, "b", "b_data"});
      pm.push_back({OUTPUT, "z", "result"});

      /* Test algorithm */
      IAlgorithm_sptr alg = std::make_shared<MockAlgorithm>();

      /* Test input properties */
      pm.setInputs(ds, alg);
      BOOST_CHECK_EQUAL(alg->getProperty(INPUT, "a"), (*ds)["a_data"]);
      BOOST_CHECK_EQUAL(alg->getProperty(INPUT, "b"), (*ds)["b_data"]);

      /* Run algorithm */
      BOOST_CHECK(alg->isValid());
      alg->execute();

      /* Test output properties */
      pm.setOutputs(ds, alg);
      BOOST_CHECK(ds->contains("result"));
      BOOST_CHECK_EQUAL(alg->getProperty(OUTPUT, "z"), (*ds)["result"]);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}

#endif
