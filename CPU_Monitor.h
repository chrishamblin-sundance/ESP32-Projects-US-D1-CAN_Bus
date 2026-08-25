#ifndef CPU_MONITOR_H
#define CPU_MONITOR_H

#include <Arduino.h>
#include <atomic>
#include "esp_freertos_hooks.h"
#include "esp_timer.h"

inline std::atomic<float> cpuCore0(0.0f);
inline std::atomic<float> cpuCore1(0.0f);

static volatile uint64_t accumulatedIdle0 = 0;
static volatile uint64_t accumulatedIdle1 = 0;
static volatile uint64_t lastIdleTime0 = 0;
static volatile uint64_t lastIdleTime1 = 0;

static bool idleHookCore0() {
  uint64_t now = esp_timer_get_time();
  uint64_t delta = now - lastIdleTime0;
  lastIdleTime0 = now;

  if (delta < 2000) {
    accumulatedIdle0 += delta;
  }
  return false;
}

static bool idleHookCore1() {
  uint64_t now = esp_timer_get_time();
  uint64_t delta = now - lastIdleTime1;
  lastIdleTime1 = now;

  if (delta < 2000) {
    accumulatedIdle1 += delta;
  }
  return false;
}

inline void setupCPUMonitor() {
  esp_register_freertos_idle_hook_for_cpu(idleHookCore0, 0);
  esp_register_freertos_idle_hook_for_cpu(idleHookCore1, 1);
}

inline void updateCPULoad() {
  static uint64_t lastCalcTime = 0;
  uint64_t now = esp_timer_get_time();
  uint64_t elapsed = now - lastCalcTime;

  if (elapsed >= 500000) { // Update every 500ms
    lastCalcTime = now;

    uint64_t idle0 = accumulatedIdle0;
    uint64_t idle1 = accumulatedIdle1;
    accumulatedIdle0 = 0;
    accumulatedIdle1 = 0;

    float load0 = 100.0f * (1.0f - ((float)idle0 / (float)elapsed));
    float load1 = 100.0f * (1.0f - ((float)idle1 / (float)elapsed));

    if (load0 < 0.0f) load0 = 0.0f;
    if (load0 > 100.0f) load0 = 100.0f;
    if (load1 < 0.0f) load1 = 0.0f;
    if (load1 > 100.0f) load1 = 100.0f;

    cpuCore0.store(load0);
    cpuCore1.store(load1);
  }
}

#endif
