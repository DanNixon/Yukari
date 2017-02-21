/** @file */

#pragma once

#include <vector>

#include "Command.h"

namespace Yukari
{
namespace CLI
{
  /**
   * @class CommandContainer
   * @author Dan Nixon
   * @brief Abstract class providing functionality required to contain and select Command.
   */
  class CommandContainer
  {
  public:
    static const size_t HELP_CMD_WIDTH;

  public:
    CommandContainer();
    virtual ~CommandContainer();

    void registerCommand(Command_sptr command);

    int handle(std::istream &in, std::ostream &out, std::vector<std::string> &tokens);
    void help(std::ostream &out);

  protected:
    std::vector<Command_sptr> m_commands; //!< Commands in this container
  };
}
}
