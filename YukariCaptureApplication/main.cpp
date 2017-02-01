#include <iostream>

#include <YukariCommon/ConfigurationManager.h>
#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;

int main()
{
  LoggingService::Init();
  auto config = ConfigurationManager::Load("config.json");
  LoggingService::Configure(config);

  /* TODO */

  std::cout << config.get<std::string>("hello.world", "nope") << '\n';

  LoggingService::GetLogger("test1")->info("hello");
  LoggingService::GetLogger("test1")->trace("hello2");
  LoggingService::GetLogger("test3")->debug("hello3");
  LoggingService::GetLogger("test3")->warn("hello4");
  LoggingService::GetLogger("test2")->error("hello5");
}
