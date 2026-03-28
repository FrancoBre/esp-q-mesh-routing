/*
 * Injection schedule loaded from LittleFS (JSON isomorphic to YAML injection-schedule).
 */
#pragma once

#include <Arduino.h>

#ifndef INJECTION_SCHEDULE_JSON_PATH
#define INJECTION_SCHEDULE_JSON_PATH "/injection-schedule.json"
#endif

enum class InjectionMode : uint8_t {
  PhysicalButtonDriven = 0,
  LoadLevel = 1,
  Periodic = 2,
};

struct InjectionConfigContext {
  InjectionMode mode = InjectionMode::PhysicalButtonDriven;
  uint32_t tick_ms = 10000;
  float load_level = 1.0f;
  /// 0 = seed RNG from runtime (non-reproducible)
  uint32_t seed = 0;
  bool loaded_from_fs = false;

  static InjectionConfigContext defaults();
};

bool loadInjectionConfigFromLittleFS(InjectionConfigContext &out);

/// Stochastic inject count per tick: E[inject] ≈ L (same logic as Java snippet).
int computeStochasticInjectCount(float L);
