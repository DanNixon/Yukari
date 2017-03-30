#include "Scheduler.h"

uint32_t Scheduler::HzToUsInterval(float hz)
{
  return (uint32_t)(1e6f / hz);
}

Scheduler::Scheduler()
    : m_numTasks(0)
{
}

int8_t Scheduler::addTask(TaskFunc f, uint32_t intervalUs)
{
  if (m_numTasks >= MAX_NUM_TASKS)
    return -1;

  m_tasks[m_numTasks].func = f;
  m_tasks[m_numTasks].intervalUs = intervalUs;
  m_tasks[m_numTasks].lastRunTimeUs = 0;
  m_tasks[m_numTasks].deltaUs = 0;

  return m_numTasks++;
}

void Scheduler::loop()
{
  uint32_t now = micros();
  int32_t delta;

  for (uint8_t i = 0; i < m_numTasks; i++)
  {
    delta = (now - m_tasks[i].lastRunTimeUs) - m_tasks[i].intervalUs;

    if (delta >= 0)
    {
      m_tasks[i].func();
      m_tasks[i].lastRunTimeUs = now;
      m_tasks[i].deltaUs = delta;
    }
  }
}

void Scheduler::print(Stream &str)
{
  str.printf("--- Tasks:\n");
  for (uint8_t i = 0; i < m_numTasks; i++)
    str.printf(" - (%d) %dus\n", m_tasks[i].func, m_tasks[i].intervalUs);
}
