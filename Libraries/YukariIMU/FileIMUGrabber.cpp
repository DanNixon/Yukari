/** @file */

#include "FileIMUGrabber.h"

#include <YukariCommon/LoggingService.h>

using namespace Yukari::Common;
using namespace Yukari::Maths;

namespace Yukari
{
namespace IMU
{

  FileIMUGrabber::FileIMUGrabber(const boost::filesystem::path &root, const std::string &pattern,
                                 const std::chrono::milliseconds &delay)
      : m_delay(delay)
  {
    FilesystemHelpers::FindByRegex(root, pattern, m_filenames);
    m_currentFile = m_filenames.cbegin();
  }

  IMUFrame::Ptr FileIMUGrabber::grabFrame()
  {
    if (m_currentFile == m_filenames.cend())
      return nullptr;

    std::this_thread::sleep_for(m_delay);

    auto retVal = std::make_shared<IMUFrame>();

    std::ifstream file(m_currentFile->string());
    file >> *retVal;

    return retVal;
  }
}
}
