/** @file */

#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "MSPClientTest"

#include <boost/test/unit_test.hpp>

#include <YukariIMU/MSPClient.h>

namespace Yukari
{
namespace IMU
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(MSPClient_Build_Payload)
    {
      MSPClient::Payload payload;
      MSPClient::Payload data = {0xff, 0xff, 0x03, 0x00, 0x70, 0x00};

      MSPClient::BuildCommandPayload(payload, MSPClient::ATTITUDE, data);

      MSPClient::Payload expected = {'$',  'M',  '<',  6,    0x6c, 0xff,
                                     0xff, 0x03, 0x00, 0x70, 0x00, 0x19};

      BOOST_CHECK_EQUAL(payload.size(), expected.size());
      BOOST_CHECK(payload == expected);
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Build_Payload_2)
    {
      MSPClient::Payload payload;

      MSPClient::BuildCommandPayload(payload, MSPClient::RAW_IMU, {});

      MSPClient::Payload expected = {'$', 'M', '<', 0, 102, 102};

      BOOST_CHECK_EQUAL(payload.size(), expected.size());
      BOOST_CHECK(payload == expected);
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Parse_Payload)
    {
      MSPClient::Payload payload = {'$',  'M',  '<',  6,    0x6c, 0xff,
                                    0xff, 0x03, 0x00, 0x70, 0x00, 0x19};
      MSPClient::MSPCommand command;
      MSPClient::Payload data;

      bool result = MSPClient::ParseResponsePayload(payload, command, data);
      BOOST_CHECK(result);

      MSPClient::Payload expected = {0xff, 0xff, 0x03, 0x00, 0x70, 0x00};

      BOOST_CHECK_EQUAL(command, MSPClient::ATTITUDE);
      BOOST_CHECK_EQUAL(data.size(), expected.size());
      BOOST_CHECK(data == expected);
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Parse_Payload_Bad_Checksum)
    {
      MSPClient::Payload payload = {'$',  'M',  '<',  6,    0x6c, 0xff,
                                    0xff, 0x03, 0x00, 0x70, 0x00, 0x10};
      MSPClient::MSPCommand command;
      MSPClient::Payload data;

      bool result = MSPClient::ParseResponsePayload(payload, command, data);
      BOOST_CHECK(!result);

      BOOST_CHECK_EQUAL(command, MSPClient::NOP);
      BOOST_CHECK(data.empty());
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Read16)
    {
      MSPClient::Payload data = {0xc9, 0xff};
      int16_t result = MSPClient::Read16(data.begin());
      BOOST_CHECK_EQUAL(result, -55);
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Parse_IMU_Payload)
    {
      /* TODO */
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Parse_Attitude_Payload)
    {
      MSPClient::Payload data = {0x4d, 0x0, 0xec, 0xff, 0x76, 0x0};
      float att[3];

      bool result = MSPClient::ParseAttitudePayload(data, att);
      BOOST_CHECK(result);

      BOOST_CHECK_EQUAL(att[0], 7.7f);
      BOOST_CHECK_EQUAL(att[1], -2.0f);
      BOOST_CHECK_EQUAL(att[2], 118.0f);
    }
  }
}
}

#endif
