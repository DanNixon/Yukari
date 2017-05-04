/** @file */

#include "IFrameProcessingTask.h"

namespace Yukari
{
namespace Processing
{
  IFrameProcessingTask::IFrameProcessingTask(const boost::filesystem::path &path)
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

  void IFrameProcessingTask::postTask(Task t)
  {
    std::lock_guard<std::mutex> lock(m_taskQueueMutex);
    m_logger->debug("Pushing task to queue (for frame {})", t.frameNumber);
    m_taskQueue.push(t);
  }

  void IFrameProcessingTask::start()
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

  void IFrameProcessingTask::stop(bool finishJobs)
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

  void IFrameProcessingTask::workerThreadFunc()
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
}
}