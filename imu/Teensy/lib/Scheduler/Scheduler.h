#pragma once

#include <Arduino.h>

class Scheduler
{
public:
  typedef void (*TaskFunc)();

  struct Task
  {
    TaskFunc func;
    uint32_t intervalUs;
    uint32_t lastRunTimeUs;
    int32_t overrunUs;
  };

public:
  static const uint8_t MAX_NUM_TASKS = 10;

public:
  static uint32_t HzToUsInterval(float hz);

public:
  Scheduler();

  int8_t addTask(TaskFunc f, uint32_t intervalUs);
  void loop();

  int32_t getOverrun(int8_t task)
  {
    return m_tasks[task].overrunUs;
  }

  void print(Stream &str);

private:
  uint8_t m_numTasks;
  Task m_tasks[MAX_NUM_TASKS];
};
