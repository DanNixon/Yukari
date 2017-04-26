/** @file */

#include "FilesystemHelpers.h"

#include "FilteredDirectoryIterator.h"

namespace Yukari
{
namespace Common
{
  std::vector<boost::filesystem::path>
  FilesystemHelpers::FindByRegex(const boost::filesystem::path &root, const std::string &pattern)
  {
    std::vector<boost::filesystem::path> matches;
    std::for_each(FilteredDirectoryIterator(".", ".*\..*"), FilteredDirectoryIterator(),
                  [&matches](const FilteredDirectoryIterator::value_type &dirEntry) {
                    matches.push_back(dirEntry.path());
                  });

    return matches;
  }
}
}