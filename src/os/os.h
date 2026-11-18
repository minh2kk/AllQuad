// AllQuad - minimal OS abstraction: periodic tasks, critical sections, shared topics.
//
// Backends:
//   os_freertos.cpp  ESP32 (native FreeRTOS, dual core) and STM32 (FreeRTOS kernel in lib/freertos)
//   os_host.cpp      deterministic cooperative scheduler for the PC simulator & tests
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace os {

using TaskFn = void (*)(void* arg);

struct TaskConfig {
  const char* name;
  TaskFn run;           // called once per period
  void* arg;
  uint32_t periodUs;    // >= 1000 uses the RTOS tick, < 1000 or the "fast" task uses a hardware timer
  uint8_t priority;     // higher = more important (1..configMAX_PRIORITIES-1)
  uint16_t stackBytes;
  int8_t core;          // ESP32: 0/1, -1 = any. Ignored elsewhere.
  bool hardwareTimed;   // drive by a hardware timer (µs-accurate) instead of the tick
};

struct TaskStats {
  const char* name;
  uint32_t periodUs;
  uint32_t lastUs;      // execution time of the last run
  uint32_t maxUs;       // worst execution time since last reset
  uint32_t overruns;    // runs that started late by more than one period
  uint32_t runs;
  uint32_t stackFree;   // bytes of stack never touched so far (FreeRTOS high-water mark), 0 = unknown
};

constexpr int MAX_TASKS = 10;

bool createTask(const TaskConfig& cfg);
// Start the hardware-timed task from the IMU's data-ready pin (rising edge) instead of the timer:
// every `divider`-th pulse of a `pulseHz` signal. Call before createTask. The pin is checked when the
// task starts (pulse rate within 25 % of pulseHz); if it is wrong, or the pulses stop later, the task
// goes back to the timer with the same period. ESP32 and STM32 (EXTI line of the pin).
void setDataReadyTrigger(int pin, uint32_t pulseHz, uint8_t divider);
// What drives the hardware-timed task: "timer", "imu-int" or "timer (imu-int lost)" ...
const char* triggerSource();
void startScheduler();          // STM32: never returns. ESP32/host: returns.
uint64_t timeUs();
uint32_t timeMs();
int taskCount();
// free heap now and the lowest it has been since boot (bytes; 0 / 0 on the PC)
void heapStats(uint32_t& freeNow, uint32_t& lowest);
TaskStats taskStats(int i);
void resetTaskStats();

// Global short critical section (spinlock on dual-core ESP32). Keep it SHORT.
void enterCritical();
void exitCritical();
struct CriticalGuard {
  CriticalGuard() { enterCritical(); }
  ~CriticalGuard() { exitCritical(); }
};

// Per-object spinlock. On the dual-core ESP32 every Topic has its own lock, so a task copying
// one topic never makes the fast loop on the other core spin on an unrelated one (the layout
// matches portMUX_TYPE, checked in os_freertos.cpp). Single-core targets just mask interrupts.
struct SpinLock {
  volatile uint32_t owner = 0xB33FFFFFu;  // SPINLOCK_FREE
  volatile uint32_t count = 0;
};
void lock(SpinLock& l);
void unlock(SpinLock& l);
struct LockGuard {
  explicit LockGuard(SpinLock& l) : l_(l) { lock(l_); }
  ~LockGuard() { unlock(l_); }
  SpinLock& l_;
};

// Single-writer / multi-reader data channel between tasks.
template <typename T>
class Topic {
 public:
  void publish(const T& v) {
    LockGuard g(lock_);
    data_ = v;
    seq_ = seq_ + 1;
  }
  // Returns the sequence number of the copied sample (0 = never published)
  uint32_t read(T& out) const {
    LockGuard g(lock_);
    out = data_;
    return seq_;
  }
  T get() const {
    T v;
    read(v);
    return v;
  }
  uint32_t seq() const { return seq_; }

 private:
  T data_{};
  volatile uint32_t seq_ = 0;
  mutable SpinLock lock_;
};

// ---- host-only helpers (simulator / unit tests) ----
#ifdef ALLQUAD_HOST
// Run all due tasks until the simulated time reaches `untilUs`.
void hostRunUntil(uint64_t untilUs);
void hostReset();
#endif

}  // namespace os
