#include "Scheduler.h"

#include <Arduino.h>

Scheduler::Scheduler()
    : m_numTasks(0)
{
}

bool Scheduler::addTask(TaskFunc f, uint32_t interval)
{
  if (m_numTasks >= MAX_NUM_TASKS)
    return false;

  m_tasks[m_numTasks].func = f;
  m_tasks[m_numTasks].interval = interval;
  m_tasks[m_numTasks].lastRunTime = 0;

  m_numTasks++;

  return true;
}

void Scheduler::loop()
{
  uint32_t now = micros();

  for (uint8_t i = 0; i < m_numTasks; i++)
  {
    if (now - m_tasks[i].lastRunTime >= m_tasks[i].interval)
    {
      m_tasks[i].func();
      m_tasks[i].lastRunTime = now;
    }
  }
}
