/** @file */

#include "FileIMUGrabber.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;

#include "boost/filesystem.hpp"
#include "boost/regex.hpp"
#include <functional>

namespace Yukari
{
namespace Common
{
  template <class NonFilteredIterator = boost::filesystem::directory_iterator>
  class FilteredDirectoryIteratorTmpl
      : public std::iterator<std::input_iterator_tag, typename NonFilteredIterator::value_type>
  {
  private:
    typedef boost::filesystem::path path;
    typedef std::function<bool(const typename NonFilteredIterator::value_type &dirEntry)>
        FilterFunction;

    NonFilteredIterator it;

    NonFilteredIterator end;

    const FilterFunction filter;

  public:
    FilteredDirectoryIteratorTmpl();

    FilteredDirectoryIteratorTmpl(const path &iteratedDir, const std::string &regexMask);

    FilteredDirectoryIteratorTmpl(const path &iteratedDir, const boost::regex &mask);

    FilteredDirectoryIteratorTmpl(const path &iteratedDir, const FilterFunction &filter);

    // preincrement
    FilteredDirectoryIteratorTmpl<NonFilteredIterator> &operator++()
    {
      for (++it; it != end && !filter(*it); ++it)
        ;
      return *this;
    };

    // postincrement
    FilteredDirectoryIteratorTmpl<NonFilteredIterator> operator++(int)
    {
      for (++it; it != end && !filter(*it); ++it)
        ;
      return FilteredDirectoryIteratorTmpl<NonFilteredIterator>(it, filter);
    };

    const boost::filesystem::directory_entry &operator*()
    {
      return *it;
    };

    bool operator!=(const FilteredDirectoryIteratorTmpl<NonFilteredIterator> &other)
    {
      return it != other.it;
    };

    bool operator==(const FilteredDirectoryIteratorTmpl<NonFilteredIterator> &other)
    {
      return it == other.it;
    };
  };

  typedef FilteredDirectoryIteratorTmpl<boost::filesystem::directory_iterator>
      FilteredDirectoryIterator;

  typedef FilteredDirectoryIteratorTmpl<boost::filesystem::recursive_directory_iterator>
      FilteredRecursiveDirectoryIterator;

  template <class NonFilteredIterator>
  FilteredDirectoryIteratorTmpl<NonFilteredIterator>::FilteredDirectoryIteratorTmpl()
      : it()
      , filter([](const boost::filesystem::directory_entry & /*dirEntry*/) { return true; })
  {
  }

  template <class NonFilteredIterator>
  FilteredDirectoryIteratorTmpl<NonFilteredIterator>::FilteredDirectoryIteratorTmpl(
      const path &iteratedDir, const std::string &regexMask)
      : FilteredDirectoryIteratorTmpl(iteratedDir, boost::regex(regexMask))
  {
  }

  template <class NonFilteredIterator>
  FilteredDirectoryIteratorTmpl<NonFilteredIterator>::FilteredDirectoryIteratorTmpl(
      const path &iteratedDir, const boost::regex &regexMask)
      : it(NonFilteredIterator(iteratedDir))
      , filter([regexMask](const boost::filesystem::directory_entry &dirEntry) {
        using std::endl;
        // return false to skip dirEntry if no match
        const std::string filename = dirEntry.path().filename().native();
        return boost::regex_match(filename, regexMask);
      })
  {
    if (it != end && !filter(*it))
      ++(*this);
  }

  template <class NonFilteredIterator>
  FilteredDirectoryIteratorTmpl<NonFilteredIterator>::FilteredDirectoryIteratorTmpl(
      const path &iteratedDir, const FilterFunction &filter)
      : it(NonFilteredIterator(iteratedDir))
      , filter(filter)
  {
    if (it != end && !filter(*it))
      ++(*this);
  }
}
}

namespace Yukari
{
namespace IMU
{
  FileIMUGrabber::FileIMUGrabber()
  {
    std::vector<std::string> all_matching_files;
    std::for_each(FilteredDirectoryIterator(".", ".*\..*"), FilteredDirectoryIterator(),
                  [&all_matching_files](const FilteredDirectoryIterator::value_type &dirEntry) {
                    all_matching_files.push_back(dirEntry.path().string());
                  });
  }

  void FileIMUGrabber::open()
  {
    m_open = true;
    m_lastFrameTime = std::chrono::high_resolution_clock::now();
  }

  void FileIMUGrabber::close()
  {
    m_open = false;
  }

  bool FileIMUGrabber::isOpen() const
  {
    return m_open;
  }

  IMUFrame::Ptr FileIMUGrabber::grabFrame()
  {
    /* Calculate timestep */
    auto timeNow = std::chrono::high_resolution_clock::now();
    auto frameDuration = timeNow - m_lastFrameTime;
    m_lastFrameTime = timeNow;

    auto retVal = std::make_shared<IMUFrame>(frameDuration);

    /* TODO */

    return retVal;
  }
}
}
