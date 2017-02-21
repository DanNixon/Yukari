/** @file */

#pragma once

#include <thread>

#include "Command.h"
#include "CommandContainer.h"

namespace Yukari
{
namespace CLI
{
  /**
   * @brief Represents reasons for termination and their associated exit codes.
   */
  enum CLIExit
  {
    CLI_RUN = -1,
    CLI_EXIT_CLEAN = 0,
    CLI_EXIT_EXCEPTION = 1
  };

  /**
   * @class CLI
   * @author Dan Nixon
   * @brief Class handling execution of a persistent command line interface.
   */
  class CLI : public CommandContainer
  {
  public:
    static const std::string PROMPT;

  public:
    CLI(std::istream &in, std::ostream &out, bool addDefaultExit = true);
    virtual ~CLI();

    int run();

    void runAsync();
    void join();

    void exit();

    /**
     * @brief Returns the exit code as of the main CLI exiting.
     * @return Exit code
     */
    inline CLIExit exitCode() const
    {
      return m_exitCode;
    }

  private:
    std::istream &m_in;
    std::ostream &m_out;

    CLIExit m_exitCode;

    std::thread m_cliThread; //!< Thread used to run the CLI async
  };
}
}
