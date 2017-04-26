/** @file */

#pragma once

#include <functional>

#include "boost/filesystem.hpp"
#include "boost/regex.hpp"

// http://stackoverflow.com/questions/1257721/can-i-use-a-mask-to-iterate-files-in-a-directory-with-boost

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

  public:
    FilteredDirectoryIteratorTmpl()
        : it()
        , filter([](const boost::filesystem::directory_entry &) { return true; })
    {
    }

    FilteredDirectoryIteratorTmpl(const path &iteratedDir, const std::string &regexMask)
        : FilteredDirectoryIteratorTmpl(iteratedDir, boost::regex(regexMask))
    {
    }

    FilteredDirectoryIteratorTmpl(const path &iteratedDir, const boost::regex &regexMask)
        : it(NonFilteredIterator(iteratedDir))
        , filter([regexMask](const boost::filesystem::directory_entry &dirEntry) {
          // return false to skip dirEntry if no match
          const std::string filename = dirEntry.path().filename().string();
          return boost::regex_match(filename, regexMask);
        })
    {
      if (it != end && !filter(*it))
        ++(*this);
    }

    FilteredDirectoryIteratorTmpl(const path &iteratedDir, const FilterFunction &filter)
        : it(NonFilteredIterator(iteratedDir))
        , filter(filter)
    {
      if (it != end && !filter(*it))
        ++(*this);
    }

    FilteredDirectoryIteratorTmpl<NonFilteredIterator> &operator++()
    {
      for (++it; it != end && !filter(*it); ++it)
        ;
      return *this;
    };

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

  private:
    NonFilteredIterator it;
    NonFilteredIterator end;

    const FilterFunction filter;
  };

  typedef FilteredDirectoryIteratorTmpl<boost::filesystem::directory_iterator>
      FilteredDirectoryIterator;

  typedef FilteredDirectoryIteratorTmpl<boost::filesystem::recursive_directory_iterator>
      FilteredRecursiveDirectoryIterator;
}
}