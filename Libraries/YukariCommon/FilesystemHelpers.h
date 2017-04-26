/** @file */

#include <map>
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
    typedef std::vector<boost::filesystem::path> PathList;

  public:
    static void FindByRegex(const boost::filesystem::path &root, const std::string &pattern,
                            PathList &out);
  };
}
}