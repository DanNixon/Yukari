#include <MSP.h>
#include <MSP_Constants.h>
#include <MSP_Data.h>
#include <MSP_Processors.h>

MSP g_msp(Serial1);

void setup()
{
  g_msp.begin();
}

void loop()
{
  g_msp.loop();
}
