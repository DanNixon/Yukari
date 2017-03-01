/** @file */

#include "ProcessingCLI.h"

#include <YukariCLI/SubCommand.h>

using namespace Yukari::CLI;
using namespace Yukari::Common;

namespace Yukari
{
namespace ProcessingApp
{
  ProcessingCLI::ProcessingCLI(std::istream &in, std::ostream &out)
      : CLI(in, out, false)
      , m_logger(LoggingService::GetLogger("ProcessingCLI"))
  {
    /* Exit command */
    registerCommand(std::make_shared<Command>(
        "exit",
        [this](std::istream &, std::ostream &, std::vector<std::string> &) {
          m_logger->trace("Clearing data store");
          m_dataStore.clear();
          exit();
          return COMMAND_EXIT_CLEAN;
        },
        0, "Exits the application."));

    /* Data command */
    {
      /* Command group */
      SubCommand_sptr data = std::make_shared<SubCommand>("data", "Manage data store.");
      registerCommand(data);

      /* List */
      data->registerCommand(std::make_shared<Command>(
          "ls",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &) {
            m_dataStore.prettyPrint(out);
            return COMMAND_EXIT_CLEAN;
          },
          0, "List contents of the data store."));

      /* Delete entry */
      data->registerCommand(std::make_shared<Command>(
          "rm",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &argv) {
            // TODO
            return COMMAND_EXIT_CLEAN;
          },
          1, "Removes enteries from the data store."));

      /* Empty */
      data->registerCommand(std::make_shared<Command>(
          "clear",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &) {
            m_dataStore.clear();
            return COMMAND_EXIT_CLEAN;
          },
          0, "Removes all contents of the data store."));
    }
  }
}
}
