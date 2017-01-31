#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;

int main()
{
  LoggingService::Configure();

  auto logger = LoggingService::GetLogger("test1");
  logger->info("hello");

  LoggingService::GetLogger("test1")->error("hello2");
  LoggingService::GetLogger("test3")->error("hello3");
  LoggingService::GetLogger("test3")->error("hello4");
  LoggingService::GetLogger("test2")->error("hello5");
}
