#include <iostream>

#include <YukariCommon/ConfigurationManager.h>
#include <YukariCommon/LoggingService.h>
#include <YukariMaths/Quaternion.h>
#include <boost/qvm/all.hpp>

using namespace Yukari::Common;
using namespace Yukari::Maths;
using namespace boost::qvm;

int main()
{
  auto config = ConfigurationManager::LoadFromAppDataDirectory("yukari", "config.json");
  LoggingService::Configure(config);

  /* TODO */

  std::cout << config.get<std::string>("hello.world", "nope") << '\n';

  LoggingService::GetLogger("test1")->info("hello");
  LoggingService::GetLogger("test1")->trace("hello2");
  LoggingService::GetLogger("test3")->debug("hello3");
  LoggingService::GetLogger("test3")->warn("hello4");
  LoggingService::GetLogger("test2")->error("hello5");

  Quaternion q(Vector3(0.0f, 1.0f, 0.0f), 90.0f, DEGREES);
  Vector3 v(1.0f, 0.0f, 0.0f);
  v = q.rotate(v);
  std::cout << v;
}
