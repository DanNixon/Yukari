#pragma once

#include <stdint.h>

class Scheduler
{
public:
  typedef void(*TaskFunc)();

  struct Task
  {
    TaskFunc func;
    uint32_t interval;
    uint32_t lastRunTime;
    int32_t overrun;
  };

public:
  static const uint8_t MAX_NUM_TASKS = 3;

public:
  Scheduler();

  int8_t addTask(TaskFunc f, uint32_t interval);
  void loop();

  int32_t getOverrun(int8_t task)
  {
    return m_tasks[task].overrun;
  }

private:
  uint8_t m_numTasks;
  Task m_tasks[MAX_NUM_TASKS];
};
