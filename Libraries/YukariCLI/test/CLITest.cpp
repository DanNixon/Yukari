#ifndef DOXYGEN_SKIP

#define BOOST_TEST_MODULE "CLITest"

#include <boost/test/unit_test.hpp>

#include <sstream>
#include <string>

#include <YukariCLI/CLI.h>
#include <YukariCLI/Command.h>
#include <YukariCLI/SubCommand.h>

namespace Yukari
{
namespace CLI
{
  namespace Test
  {
    BOOST_AUTO_TEST_CASE(CLI_Prompt_Help_Exit)
    {
      // Simulated input
      std::stringstream in("help\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out);

      const std::string expected = "> "
                                   "Command usage:\n"
                                   " help                : Shows command usage.\n"
                                   " exit                : Exit the application.\n"
                                   "> ";

      BOOST_CHECK_EQUAL(0, c.run());
      BOOST_CHECK_EQUAL(expected, out.str());
    }

    BOOST_AUTO_TEST_CASE(CLI_MissingCommand)
    {
      // Simulated input
      std::stringstream in("nope\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out);

      const std::string expected = "> "
                                   "Command \"nope\" not found.\n"
                                   "> ";

      BOOST_CHECK_EQUAL(0, c.run());
      BOOST_CHECK_EQUAL(expected, out.str());
    }

    BOOST_AUTO_TEST_CASE(CLI_TooFewArguments)
    {
      // Simulated input
      std::stringstream in("test 111\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out);

      c.registerCommand(Command_ptr(
          new Command("test",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command.\n";
                        return 0;
                      },
                      3, "Is a test.")));

      const std::string expected = "> "
                                   "Too few arguments (got 2, expected 3).\n"
                                   "> ";

      BOOST_CHECK_EQUAL(0, c.run());
      BOOST_CHECK_EQUAL(expected, out.str());
    }

    BOOST_AUTO_TEST_CASE(CLI_SubCommand)
    {
      // Simulated input
      std::stringstream in("test\nsub1 help\nsub1 list\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out);

      c.registerCommand(Command_ptr(
          new Command("test",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command.\n";
                        return 0;
                      },
                      0, "Is a test.")));

      SubCommand_ptr sub1(new SubCommand("sub1", "Test subcommand."));

      sub1->registerCommand(Command_ptr(
          new Command("list",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command => list.\n";
                        return 0;
                      },
                      0, "Is a test 2.")));

      c.registerCommand(sub1);

      const std::string expected = "> "
                                   "Test command.\n"
                                   "> "
                                   "Command usage:\n"
                                   " help                : Shows command usage.\n"
                                   " list                : Is a test 2.\n"
                                   "> "
                                   "Test command => list.\n"
                                   "> ";

      BOOST_CHECK_EQUAL(0, c.run());
      BOOST_CHECK_EQUAL(expected, out.str());
    }

    BOOST_AUTO_TEST_CASE(CLI_SubCommand_Disabled)
    {
      // Simulated input
      std::stringstream in("help\ntest\nsub1 help\nsub1 list\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out);

      c.registerCommand(Command_ptr(
          new Command("test",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command.\n";
                        return 0;
                      },
                      0, "Is a test.")));

      SubCommand_ptr sub1(new SubCommand("sub1", "Test subcommand."));

      // Disable sub1 command
      sub1->setEnabled(false);

      sub1->registerCommand(Command_ptr(
          new Command("list",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command => list.\n";
                        return 0;
                      },
                      0, "Is a test 2.")));

      c.registerCommand(sub1);

      // sub1 command (and its sub commands) should never work
      const std::string expected = "> "
                                   "Command usage:\n"
                                   " help                : Shows command usage.\n"

                                   " exit                : Exit the application.\n"
                                   " test                : Is a test.\n"
                                   "> "
                                   "Test command.\n"
                                   "> "
                                   "Command \"sub1\" not found.\n"
                                   "> "
                                   "Command \"sub1\" not found.\n"
                                   "> ";

      BOOST_CHECK_EQUAL(0, c.run());
      BOOST_CHECK_EQUAL(expected, out.str());
    }
  };
}
}

#endif