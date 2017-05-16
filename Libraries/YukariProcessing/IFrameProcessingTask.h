/** @file */

#pragma once

#include <atomic>
#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMUFrame.h>

namespace Yukari
{
namespace Processing
{
  class IFrameProcessingTask
  {
  public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef std::shared_ptr<IFrameProcessingTask> Ptr;

  public:
    struct Task
    {
      size_t frameNumber;
      CloudConstPtr cloud;
      IMU::IMUFrame::Ptr imuFrame;
    };

  public:
    IFrameProcessingTask(const boost::filesystem::path &path = boost::filesystem::path(""));

    void postTask(Task t);

    void start();
    void stop(bool finishJobs = true);

    inline boost::filesystem::path formatFilename(Task t, const std::string &filename) const
    {
      std::stringstream ss;
      ss << std::setw(5) << std::setfill('0') << t.frameNumber;
      return m_outputDirectory / (ss.str() + filename);
    }

    inline bool isRunning() const
    {
      return m_run.load();
    }

    inline size_t queueLength()
    {
      std::lock_guard<std::mutex> lock(m_taskQueueMutex);
      return m_taskQueue.size();
    }

  private:
    void workerThreadFunc();

  protected:
    virtual int onStart()
    {
      return 0;
    }

    virtual int process(Task t) = 0;

    virtual int onStop()
    {
      return 0;
    }

  private:
    Common::LoggingService::Logger m_logger;

    std::thread m_workerThread;
    std::atomic_bool m_run;
    std::atomic_bool m_terminate;

    std::mutex m_taskQueueMutex;
    std::queue<Task> m_taskQueue;

  protected:
    boost::filesystem::path m_outputDirectory;
  };
}
}
