#ifndef DOXYGEN_SKIP

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
    BOOST_AUTO_TEST_SUITE(CLITest)

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

    BOOST_AUTO_TEST_CASE(CLI_Missing_Command)
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

    BOOST_AUTO_TEST_CASE(CLI_Too_Few_Arguments)
    {
      // Simulated input
      std::stringstream in("test 111 222\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out);

      c.registerCommand(Command_sptr(
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

      c.registerCommand(Command_sptr(
          new Command("test",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command.\n";
                        return 0;
                      },
                      0, "Is a test.")));

      SubCommand_sptr sub1(new SubCommand("sub1", "Test subcommand."));

      sub1->registerCommand(Command_sptr(
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

      c.registerCommand(Command_sptr(
          new Command("test",
                      [](std::istream &, std::ostream &out, std::vector<std::string> &) {
                        out << "Test command.\n";
                        return 0;
                      },
                      0, "Is a test.")));

      SubCommand_sptr sub1(new SubCommand("sub1", "Test subcommand."));

      // Disable sub1 command
      sub1->setEnabled(false);

      sub1->registerCommand(Command_sptr(
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

    BOOST_AUTO_TEST_CASE(CLI_Alternate_Input)
    {
      // Simulated input
      std::stringstream in("say hello1\ntest\nsay hello3\nexit\n");

      // Capture output
      std::stringstream out;

      CLI c(in, out, true, true);

      c.registerCommand(std::make_shared<Command>(
          "say",
          [&c](std::istream &, std::ostream &out, std::vector<std::string> &argv) {
            out << argv[1] << '\n';
            return COMMAND_EXIT_CLEAN;
          },
          1));

      c.registerCommand(std::make_shared<Command>(
          "test",
          [&c](std::istream &, std::ostream &out, std::vector<std::string> &) {
            std::shared_ptr<std::istream> s = std::make_shared<std::stringstream>();
            *std::dynamic_pointer_cast<std::stringstream>(s) << "say hello2\nscript reset\n";
            c.redirectInput(s);
            out << "Input redirected\n";
            return COMMAND_EXIT_CLEAN;
          },
          0));

      const std::string expected = "> "
                                   "hello1\n"
                                   "> "
                                   "Input redirected\n"
                                   "> "
                                   "hello2\n"
                                   "> "
                                   "> "
                                   "hello3\n"
                                   "> ";

      BOOST_CHECK_EQUAL(0, c.run());
      BOOST_CHECK_EQUAL(expected, out.str());
    }

    BOOST_AUTO_TEST_SUITE_END()
  };
}
}

#endif
