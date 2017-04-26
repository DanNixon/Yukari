/** @file */

#include "FilesystemHelpers.h"

#include "FilteredDirectoryIterator.h"

namespace Yukari
{
namespace Common
{
  void FilesystemHelpers::FindByRegex(const boost::filesystem::path &root,
                                      const std::string &pattern, PathList &out)
  {
    std::for_each(FilteredDirectoryIterator(boost::filesystem::absolute(root), pattern), FilteredDirectoryIterator(),
                  [&out](const FilteredDirectoryIterator::value_type &dirEntry) {
                    out.push_back(dirEntry.path());
                  });
  }
}
}
