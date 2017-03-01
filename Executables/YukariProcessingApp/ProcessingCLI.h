/** @file */

#pragma once

#include <YukariCLI/CLI.h>

#include <YukariProcessing/DataStore.h>

namespace Yukari
{
namespace ProcessingApp
{
  class ProcessingCLI : public CLI::CLI
  {
  public:
    ProcessingCLI(std::istream &in, std::ostream &out);

  private:
    Common::LoggingService::Logger m_logger;
    Processing::DataStore m_dataStore;
  };
}
}
