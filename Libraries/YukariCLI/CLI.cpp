/** @file */

#include "CLI.h"

#include <string>

#include <boost/algorithm/string.hpp>

#include "SubCommand.h"

namespace Yukari
{
namespace CLI
{
  /**
   * @brief String that is shown when the CLI prompt is displayed.
   */
  const std::string CLI::PROMPT = "> ";

  /**
   * @brief Create a new command line on given streams.
   * @param in Input stream
   * @param out Output stream
   * @param addDefaultExit If the default exit command should be added (defaults to true)
   *
   * Implicitly adds the "exit" command to exit the application.
   */
  CLI::CLI(std::istream &in, std::ostream &out, bool addDefaultExit, bool addScriptingCommands)
      : CommandContainer()
      , m_in(in)
      , m_out(out)
      , m_exitCode(CLI_RUN)
  {
    if (addDefaultExit)
    {
      /* Add exit command */
      m_commands.push_back(std::make_shared<Command>(
          "exit",
          [this](std::istream &, std::ostream &, std::vector<std::string> &) {
            exit();
            return COMMAND_EXIT_CLEAN;
          },
          0, "Exit the application."));
    }

    if (addScriptingCommands)
    {
      /* Add scripting commands */
      SubCommand_sptr scripting = std::make_shared<SubCommand>("script", "Work with script files");
      m_commands.push_back(scripting);

      /* Run script command */
      scripting->registerCommand(std::make_shared<Command>(
          "run", [](std::istream &, std::ostream &,
                    std::vector<std::string> &argv) { return COMMAND_EXIT_CLEAN; },
          1, "Run a command line script."));

      /* Exit scripting command */
      scripting->registerCommand(std::make_shared<Command>(
          "reset",
          [this](std::istream &, std::ostream &, std::vector<std::string> &argv) {
            resetInput();
            return COMMAND_EXIT_CLEAN;
          },
          0, "Resets the input stream (must be called at the end of a script)."));
    }
  }

  CLI::~CLI()
  {
  }

  /**
   * @brief Runs the command line interface.
   * @return Exit code
   *
   * This should be called in main() after required setup logic.
   */
  int CLI::run()
  {
    while (m_exitCode == CLI_RUN)
    {
      m_out << PROMPT;

      std::istream &in = (m_alternateIn) ? *m_alternateIn : m_in;

      std::string command;
      std::getline(in, command);

      std::vector<std::string> tokens;
      boost::algorithm::split(tokens, command, boost::algorithm::is_any_of(" "));

      int retVal = handle(in, m_out, tokens);

      if (retVal > 0)
        m_out << "Command exited with code " << retVal << ".\n";
    }

    return (int)m_exitCode;
  }

  /**
   * @brief Starts the CLI in a new thread.
   */
  void CLI::runAsync()
  {
    m_cliThread = std::thread(&CLI::run, this);
  }

  /**
   * @brief Joins the CLI async thread.
   */
  void CLI::join()
  {
    if (m_cliThread.joinable())
      m_cliThread.join();
  }

  /**
   * @brief Sets the flag to exit the application cleanly.
   */
  void CLI::exit()
  {
    m_exitCode = CLI_EXIT_CLEAN;
  }
}
}
