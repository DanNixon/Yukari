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
      : m_logger(LoggingService::Instance().getLogger("FileIMUGrabber"))
      , m_delay(delay)
  {
    FilesystemHelpers::FindByRegex(root, pattern, m_filenames);
    m_currentFile = m_filenames.cbegin();
  }

  IMUFrame::Ptr FileIMUGrabber::grabFrame()
  {
    if (m_currentFile == m_filenames.cend())
    {
      m_logger->error("No more saved IMU frames");
      return nullptr;
    }

    m_logger->trace("Delay start... ({}ms)", m_delay.count());
    std::this_thread::sleep_for(m_delay);
    m_logger->trace("Delay end");

    auto retVal = std::make_shared<IMUFrame>();

    m_logger->debug("Reading file: \"{}\"", m_currentFile->string());
    std::ifstream file(m_currentFile->string());
    file >> *retVal;

    m_currentFile++;

    return retVal;
  }
}
}
