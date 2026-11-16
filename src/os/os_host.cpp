// AllQuad - deterministic cooperative scheduler used by the simulator and tests.
// Tasks run to completion; when several are due at the same instant the one
// with the highest priority runs first, like a preemptive RTOS would.
#ifdef ALLQUAD_HOST
#include <string.h>

#include "os.h"

namespace os {

namespace {
struct HostTask {
  TaskConfig cfg;
  uint64_t nextUs;
  TaskStats stats;
};
HostTask tasks[MAX_TASKS];
int nTasks = 0;
uint64_t nowUs = 0;
}  // namespace

bool createTask(const TaskConfig& cfg) {
  if (nTasks >= MAX_TASKS) return false;
  HostTask& t = tasks[nTasks++];
  t.cfg = cfg;
  t.nextUs = nowUs + cfg.periodUs;
  memset(&t.stats, 0, sizeof(t.stats));
  t.stats.name = cfg.name;
  t.stats.periodUs = cfg.periodUs;
  return true;
}

void startScheduler() {}
void setDataReadyTrigger(int, uint32_t, uint8_t) {}
const char* triggerSource() { return "timer"; }
uint64_t timeUs() { return nowUs; }
uint32_t timeMs() { return (uint32_t)(nowUs / 1000); }
int taskCount() { return nTasks; }
TaskStats taskStats(int i) { return tasks[i].stats; }
void resetTaskStats() {
  for (int i = 0; i < nTasks; i++) tasks[i].stats.maxUs = 0;
}
void lock(SpinLock&) {}
void unlock(SpinLock&) {}
void enterCritical() {}
void exitCritical() {}

void hostRunUntil(uint64_t untilUs) {
  for (;;) {
    int best = -1;
    for (int i = 0; i < nTasks; i++) {
      if (tasks[i].nextUs > untilUs) continue;
      if (best < 0 || tasks[i].nextUs < tasks[best].nextUs ||
          (tasks[i].nextUs == tasks[best].nextUs && tasks[i].cfg.priority > tasks[best].cfg.priority))
        best = i;
    }
    if (best < 0) break;
    HostTask& t = tasks[best];
    nowUs = t.nextUs;
    t.cfg.run(t.cfg.arg);
    t.stats.runs++;
    t.nextUs += t.cfg.periodUs;
  }
  nowUs = untilUs;
}

void hostReset() {
  nTasks = 0;
  nowUs = 0;
}

}  // namespace os
#endif
