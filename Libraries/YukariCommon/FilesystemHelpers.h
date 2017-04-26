/** @file */

#include <string>
#include <vector>

#include <boost/filesystem.hpp>

namespace Yukari
{
namespace Common
{
  class FilesystemHelpers
  {
  public:
    static std::vector<boost::filesystem::path> FindByRegex(const boost::filesystem::path &root,
                                                            const std::string &pattern);
  };
}
}