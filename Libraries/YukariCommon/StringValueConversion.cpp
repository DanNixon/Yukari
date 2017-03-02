/** @file */

#include "StringValueConversion.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/trim_all.hpp>

#include "LoggingService.h"

namespace Yukari
{
namespace Common
{
  boost::any StringValueConversion::Convert(std::string type, const std::string value)
  {
    /* Clean type name */
    boost::algorithm::to_lower(type);
    boost::algorithm::trim(type);

    /* Do conversion */
    if (type == "string")
      return boost::any(value);
    else if (type == "int")
      return boost::any(std::stoi(value));
    else if (type == "float")
      return boost::any(std::stof(value));
    else
      throw std::runtime_error("Unknown type: \"" + type + "\"");

    return boost::any();
  }

  std::vector<boost::any> StringValueConversion::Convert(std::string type,
                                                         const std::vector<std::string> &values)
  {
    std::vector<boost::any> retVal;

    for (auto it = values.cbegin(); it != values.cend(); ++it)
      retVal.push_back(Convert(type, *it));

    return retVal;
  }
}
}
