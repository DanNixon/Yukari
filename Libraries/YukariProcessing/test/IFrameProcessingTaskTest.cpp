/** @file */

#include <string>
#include <vector>

#include <boost/test/unit_test.hpp>
#include <pcl/point_types.h>

#include <YukariProcessing/IFrameProcessingTask.h>

namespace Yukari
{
namespace Processing
{
  class MockTask : public IFrameProcessingTask<pcl::PointXYZ>
  {
  public:
    MockTask()
        : IFrameProcessingTask()
    {
    }

  protected:
    virtual int onStart() override
    {
      m_callQueue.push_back("start");
      return 0;
    }

    virtual int process(Task t) override
    {
      m_callQueue.push_back("process " + std::to_string(t.frameNumber));

      /* Sleep for the frame number milliseconds, simulates doing work */
      std::this_thread::sleep_for(std::chrono::milliseconds(t.frameNumber));

      return 0;
    }

    virtual int onStop() override
    {
      m_callQueue.push_back("stop");
      return 0;
    }

  public:
    std::vector<std::string> m_callQueue;
  };

  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(IFrameProcessingTaskTest)

    BOOST_AUTO_TEST_CASE(IFrameProcessingTask_Basic)
    {
      MockTask t;
      BOOST_CHECK(!t.isRunning());

      /* Start worker */
      t.start();
      BOOST_CHECK(t.isRunning());

      /* Add some tasks */
      t.postTask({50, nullptr, nullptr});
      t.postTask({100, nullptr, nullptr});

      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      /* Stop and join worker */
      t.stop();
      BOOST_CHECK(!t.isRunning());

      /* Check operations */
      BOOST_CHECK_EQUAL(t.m_callQueue.size(), 4);
      BOOST_CHECK_EQUAL(t.m_callQueue.front(), std::string("start"));
      BOOST_CHECK_EQUAL(t.m_callQueue.back(), std::string("stop"));
      BOOST_CHECK_EQUAL(t.m_callQueue[1], std::string("process 50"));
      BOOST_CHECK_EQUAL(t.m_callQueue[2], std::string("process 100"));
    }

    BOOST_AUTO_TEST_CASE(IFrameProcessingTask_2)
    {
      MockTask t;
      BOOST_CHECK(!t.isRunning());

      /* Add some tasks */
      t.postTask({50, nullptr, nullptr});
      t.postTask({100, nullptr, nullptr});

      /* Start worker */
      t.start();
      BOOST_CHECK(t.isRunning());

      /* Add some more tasks */
      t.postTask({25, nullptr, nullptr});
      t.postTask({88, nullptr, nullptr});

      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      /* Stop and join worker */
      t.stop();
      BOOST_CHECK(!t.isRunning());

      /* Check operations */
      BOOST_CHECK_EQUAL(t.m_callQueue.size(), 6);
      BOOST_CHECK_EQUAL(t.m_callQueue.front(), std::string("start"));
      BOOST_CHECK_EQUAL(t.m_callQueue.back(), std::string("stop"));
      BOOST_CHECK_EQUAL(t.m_callQueue[1], std::string("process 50"));
      BOOST_CHECK_EQUAL(t.m_callQueue[2], std::string("process 100"));
      BOOST_CHECK_EQUAL(t.m_callQueue[3], std::string("process 25"));
      BOOST_CHECK_EQUAL(t.m_callQueue[4], std::string("process 88"));
    }

    BOOST_AUTO_TEST_SUITE_END()
  };
}
}
