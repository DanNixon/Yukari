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
  };

public:
  static const uint8_t MAX_NUM_TASKS = 3;

public:
  Scheduler();

  bool addTask(TaskFunc f, uint32_t interval);

  void loop();

private:
  uint8_t m_numTasks;
  Task m_tasks[MAX_NUM_TASKS];
};
