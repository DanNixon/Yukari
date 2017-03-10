/** @file */

#include "ProcessingCLI.h"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <YukariAlgorithms/AlgorithmFactory.h>
#include <YukariCLI/SubCommand.h>
#include <YukariCommon/StringValueConversion.h>
#include <YukariProcessing/PropertyMapping.h>

using namespace Yukari::Algorithms;
using namespace Yukari::CLI;
using namespace Yukari::Common;
using namespace Yukari::Processing;

namespace Yukari
{
namespace ProcessingApp
{
  ProcessingCLI::ProcessingCLI(std::istream &in, std::ostream &out)
      : CLI(in, out, false, true)
      , m_logger(LoggingService::GetLogger("ProcessingCLI"))
      , m_dataStore(std::make_shared<DataStore>())
  {
    /* Exit command */
    registerCommand(std::make_shared<Command>(
        "exit",
        [this](std::istream &, std::ostream &, std::vector<std::string> &) {
          m_logger->trace("Clearing data store");
          m_dataStore->clear();
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
          [this](std::istream &, std::ostream &out, std::vector<std::string> &argv) {
            if (argv.size() > 1)
              m_dataStore->prettyPrint(out, boost::regex(argv[1]));
            else
              m_dataStore->prettyPrint(out);

            return COMMAND_EXIT_CLEAN;
          },
          0, "List contents of the data store."));

      /* Add property */
      data->registerCommand(std::make_shared<Command>(
          "add",
          [this](std::istream &, std::ostream &, std::vector<std::string> &argv) {
            Property_sptr p = m_dataStore->addNewProperty(argv[1], std::stoi(argv[2]));
            return ((p) ? COMMAND_EXIT_CLEAN : 1);
          },
          2, "Adds a new property [name] [size]."));

      /* Set value */
      data->registerCommand(std::make_shared<Command>(
          "set",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &argv) -> int {
            /* Parse values */
            std::vector<std::string> valuesStrs;
            boost::algorithm::split(valuesStrs, argv[3], boost::is_any_of(","));
            std::vector<boost::any> values = StringValueConversion::Convert(argv[2], valuesStrs);

            /* Get property */
            DataStore::ItemList properties = m_dataStore->findByRegex(boost::regex(argv[1]));
            if (properties.size() > 1)
            {
              out << "Ambiguous property.\n";
              return 1;
            }
            else if (properties.empty())
            {
              out << "No property found.\n";
              return 1;
            }

            /* Set values */
            properties.front().second->set(values);

            return COMMAND_EXIT_CLEAN;
          },
          3, "Sets the value of a property [name] [type] [values]."));

      /* Get value */
      data->registerCommand(std::make_shared<Command>(
          "get",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &argv) -> int {
            /* Get property */
            DataStore::ItemList properties = m_dataStore->findByRegex(boost::regex(argv[1]));
            if (properties.size() > 1)
            {
              out << "Ambiguous property.\n";
              return 1;
            }
            else if (properties.empty())
            {
              out << "No property found.\n";
              return 1;
            }

            Property_sptr p = properties.front().second;

            /* Parse and print values */
            for (auto it = p->cbegin(); it != p->cend(); ++it)
            {
              if (it != p->cbegin())
                std::cout << ',';

              std::cout << StringValueConversion::Convert(argv[2], *it);
            }
            std::cout << '\n';

            return COMMAND_EXIT_CLEAN;
          },
          2, "Sets the value of a property [name] [type]."));

      /* Delete entry */
      data->registerCommand(std::make_shared<Command>(
          "rm",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &argv) {
            std::vector<std::string> removed = m_dataStore->deleteByRegex(boost::regex(argv[1]));
            if (!removed.empty())
            {
              out << "Removed " << removed.size() << " items: ";
              for (auto it = removed.begin(); it != removed.end(); ++it)
              {
                if (it != removed.begin())
                  out << ", ";
                out << *it;
              }
              out << '\n';
            }
            return COMMAND_EXIT_CLEAN;
          },
          1, "Removes enteries from the data store."));

      /* Empty */
      data->registerCommand(std::make_shared<Command>(
          "clear",
          [this](std::istream &, std::ostream &, std::vector<std::string> &) {
            m_dataStore->clear();
            return COMMAND_EXIT_CLEAN;
          },
          0, "Removes all contents of the data store."));
    }

    /* Algorithm command */
    {
      /* Command group */
      SubCommand_sptr alg = std::make_shared<SubCommand>("alg", "Work with algorithms.");
      registerCommand(alg);

      /* Run */
      alg->registerCommand(std::make_shared<Command>(
          "run",
          [this](std::istream &, std::ostream &out, std::vector<std::string> &args) -> int {
            /* Create algorithm */
            IAlgorithm_sptr alg = AlgorithmFactory::Create(args[1]);
            if (!alg)
            {
              out << "Failed to create algorithm with name \"" << args[1] << "\"\n";
              return 1;
            }

            /* Get properties */
            PropertyMapping pMap;
            pMap.parseString(args.begin() + 2, args.end());

            /* Set algorithm inputs as per mapping */
            pMap.setInputs(m_dataStore, alg);

            /* Validate algorithm */
            auto result = alg->validate();
            if (!result.empty())
            {
              out << "Algorithm validation failed!\n";
              for (auto it = result.begin(); it != result.end(); ++it)
                out << '[' << it->first << "] " << it->second << '\n';

              return 2;
            }

            /* Run algorithm */
            alg->execute();

            /* Put results in data store as per mapping */
            pMap.setOutputs(m_dataStore, alg);

            return COMMAND_EXIT_CLEAN;
          },
          1, "Runs an algorithm."));
    }
  }
}
}