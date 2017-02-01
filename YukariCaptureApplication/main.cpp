#include <iostream>

#include <YukariCommon/LoggingService.h>
#include <YukariCommon/ConfigurationManager.h>

using namespace Yukari::Common;

int main()
{
  LoggingService::Init();

  auto pt = ConfigurationManager::Load("test.ini");

  std::cout << pt.get<std::string>("hello.world", "nope") << '\n';

  LoggingService::Configure();

  auto logger = LoggingService::GetLogger("test1");
  logger->info("hello");

  LoggingService::GetLogger("test1")->error("hello2");
  LoggingService::GetLogger("test3")->error("hello3");
  LoggingService::GetLogger("test3")->error("hello4");
  LoggingService::GetLogger("test2")->error("hello5");
}
