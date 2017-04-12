#include <iostream>

#include <YukariCommon/LoggingService.h>

int main()
{
  auto logger = Yukari::Common::LoggingService::Instance().getLogger("PointCloudAlignment");

  /* TODO */

  logger->info("Exiting");
  return 0;
}
