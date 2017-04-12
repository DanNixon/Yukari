#include <iostream>

#include <YukariCommon/LoggingService.h>

int main()
{
  auto logger = Yukari::Common::LoggingService::GetLogger("PointCloudAlignment");

  /* TODO */

  logger->info("Exiting");
  return 0;
}
