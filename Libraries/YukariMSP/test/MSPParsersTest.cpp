/** @file */

#include <boost/test/unit_test.hpp>

#include <YukariMSP/MSPParsers.h>

namespace Yukari
{
namespace MSP
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(MSPClientTest)

    BOOST_AUTO_TEST_CASE(MSPClient_Read16)
    {
      MSPClient::Payload data = {0xc9, 0xff};
      int16_t result = MSPParsers::Read16(data.begin());
      BOOST_CHECK_EQUAL(result, -55);
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Parse_IMU_Payload)
    {
      MSPClient::Payload data = {0x27, 0x0,  0x50, 0x0, 0x5, 0x2, 0xfa, 0xff, 0xfb,
                                 0xff, 0xfe, 0xff, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0};
      int16_t gyro[3];
      int16_t acc[3];
      int16_t mag[3];

      bool result = MSPParsers::ParseRawIMUPayload(data, gyro, acc, mag);
      BOOST_CHECK(result);

      BOOST_CHECK_EQUAL(gyro[0], -6);
      BOOST_CHECK_EQUAL(gyro[1], -5);
      BOOST_CHECK_EQUAL(gyro[2], -2);

      BOOST_CHECK_EQUAL(acc[0], 39);
      BOOST_CHECK_EQUAL(acc[1], 80);
      BOOST_CHECK_EQUAL(acc[2], 517);

      BOOST_CHECK_EQUAL(mag[0], 0);
      BOOST_CHECK_EQUAL(mag[1], 0);
      BOOST_CHECK_EQUAL(mag[2], 0);
    }

    BOOST_AUTO_TEST_CASE(MSPClient_Parse_Attitude_Payload)
    {
      MSPClient::Payload data = {0x4d, 0x0, 0xec, 0xff, 0x76, 0x0};
      float att[3];

      bool result = MSPParsers::ParseAttitudePayload(data, att);
      BOOST_CHECK(result);

      BOOST_CHECK_EQUAL(att[0], 7.7f);
      BOOST_CHECK_EQUAL(att[1], -2.0f);
      BOOST_CHECK_EQUAL(att[2], 118.0f);
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}
