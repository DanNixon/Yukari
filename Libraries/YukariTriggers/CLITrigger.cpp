/** @file */

#include "CLITrigger.h"

using namespace Yukari::CLI;

namespace Yukari
{
namespace Triggers
{
  CLITrigger::CLITrigger(const std::string &cmdName, const std::string &description)
      : m_cliCommand(std::make_shared<Command>(
            cmdName,
            [this](std::istream &, std::ostream &, std::vector<std::string> &) {
              this->trigger();
              return CommandExit::COMMAND_EXIT_CLEAN;
            },
            0, description))
  {
  }

  void CLITrigger::enable()
  {
    if (m_cliCommand)
      m_cliCommand->setEnabled(true);
  }

  void CLITrigger::disable()
  {
    if (m_cliCommand)
      m_cliCommand->setEnabled(false);
  }

  void CLITrigger::trigger()
  {
    if (m_handlerFunc)
      m_handlerFunc();
  }
}
}
