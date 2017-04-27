/** @file */

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>

#include <YukariCommon/LoggingService.h>
#include <YukariIMU/IMUFrame.h>

namespace Yukari
{
namespace Processing
{
  template <typename POINT_TYPE> class IFrameProcessingTask
  {
  public:
    typedef pcl::PointCloud<POINT_TYPE> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    typedef std::shared_ptr<IFrameProcessingTask<POINT_TYPE>> Ptr;

  public:
    struct Task
    {
      size_t frameNumber;
      CloudConstPtr cloud;
      IMU::IMUFrame::Ptr imuFrame;
    };

  public:
    IFrameProcessingTask(const boost::filesystem::path &path = boost::filesystem::path(""))
        : m_logger(Common::LoggingService::Instance().getLogger("IFrameProcessingTask"))
        , m_outputDirectory(path)
    {
      /* Set running flag */
      m_run.store(false);
      m_terminate.store(false);

      /* Ensure capture directory exists if it has been provided */
      if (!m_outputDirectory.empty())
      {
        m_logger->info("Capture path: {}", m_outputDirectory);
        boost::filesystem::create_directories(m_outputDirectory);
        m_logger->debug("Capture root directory created");
      }
      else
      {
        m_logger->trace("No output directory");
      }
    }

    void postTask(Task t)
    {
      std::lock_guard<std::mutex> lock(m_taskQueueMutex);
      m_logger->debug("Pushing task to queue (for frame {})", t.frameNumber);
      m_taskQueue.push(t);
    }

    void start()
    {
      if (m_run.load())
      {
        m_logger->warn("Task worker already running, cannot start");
        return;
      }

      /* Fire start handler */
      int result = onStart();
      m_logger->trace("onStart result: {}", result);

      /* Set run flag */
      m_run.store(true);
      m_terminate.store(false);

      /* Start worker thread */
      m_workerThread = std::thread(&IFrameProcessingTask::workerThreadFunc, this);
    }

    void stop(bool finishJobs = true)
    {
      if (!m_run.load())
      {
        m_logger->warn("Task worker is not running, cannot stop");
        return;
      }

      /* Clear run flag */
      if (finishJobs)
      {
        m_logger->debug("Will wait for jobs to finish");
        m_terminate.store(true);
      }
      else
      {
        m_logger->debug("Will terminate jobs");
        m_run.store(false);
      }

      /* Join worker thread */
      if (m_workerThread.joinable())
        m_workerThread.join();

      /* Fire stop handler */
      int result = onStop();
      m_logger->trace("onStop result: {}", result);
    }

    inline bool isRunning() const
    {
      return m_run.load();
    }

    inline size_t queueLength() const
    {
      std::lock_guard<std::mutex> lock(m_taskQueueMutex);
      return m_taskQueue.size();
    }

  private:
    void workerThreadFunc()
    {
      while (m_run.load())
      {
        Task t;

        {
          std::lock_guard<std::mutex> lock(m_taskQueueMutex);

          /* Skip an empty queue */
          if (m_taskQueue.empty())
          {
            /* If the queue is empty and we want to finish jobs then we are done */
            if (m_terminate.load())
            {
              m_logger->trace("All jobs done, worker will exit now");
              m_run.store(false);
            }

            continue;
          }

          /* Retrieve the task data */
          t = m_taskQueue.front();
          m_taskQueue.pop();

          m_logger->debug("{} task(s) remiaining in queue", m_taskQueue.size());
        }

        /* Process the task */
        m_logger->trace("Processing task started (for frame {})", t.frameNumber);
        process(t);
        m_logger->trace("Processing task complete (for frame {})", t.frameNumber);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

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
