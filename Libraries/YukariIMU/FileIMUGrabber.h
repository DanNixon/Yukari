/** @file */

#pragma once

#include "IIMUGrabber.h"

#include <chrono>

#include <boost/filesystem.hpp>

#include <YukariCommon/FilesystemHelpers.h>

namespace Yukari
{
namespace IMU
{
  class FileIMUGrabber : public IIMUGrabber
  {
  public:
    FileIMUGrabber(const boost::filesystem::path &root, const std::string &pattern,
                   const std::chrono::milliseconds &delay = std::chrono::milliseconds(10));

    virtual IMUFrame::Ptr grabFrame() override;

  protected:
    std::chrono::milliseconds m_delay;

    Common::FilesystemHelpers::PathList m_filenames;
    Common::FilesystemHelpers::PathList::const_iterator m_currentFile;
  };
}
}
