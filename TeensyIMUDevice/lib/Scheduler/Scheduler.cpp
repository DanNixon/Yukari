#include "Scheduler.h"

#include <Arduino.h>

Scheduler::Scheduler()
    : m_numTasks(0)
{
}

int8_t Scheduler::addTask(TaskFunc f, uint32_t interval)
{
  if (m_numTasks >= MAX_NUM_TASKS)
    return -1;

  m_tasks[m_numTasks].func = f;
  m_tasks[m_numTasks].interval = interval;
  m_tasks[m_numTasks].lastRunTime = 0;

  return m_numTasks++;
}

void Scheduler::loop()
{
  uint32_t now = micros();
  int32_t delta;

  for (uint8_t i = 0; i < m_numTasks; i++)
  {
    delta = (now - m_tasks[i].lastRunTime) - m_tasks[i].interval;

    if (delta >= 0)
    {
      m_tasks[i].func();
      m_tasks[i].lastRunTime = now;
      m_tasks[i].delta = delta;
    }
  }
}
