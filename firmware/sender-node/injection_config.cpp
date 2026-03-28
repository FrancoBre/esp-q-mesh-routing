#include "injection_config.h"

#include <LittleFS.h>
#include <cmath>

InjectionConfigContext InjectionConfigContext::defaults() {
  InjectionConfigContext c;
  c.mode = InjectionMode::PhysicalButtonDriven;
  c.tick_ms = 10000;
  c.load_level = 1.0f;
  c.seed = 0;
  c.loaded_from_fs = false;
  return c;
}

static bool parseModeString(const char *s, InjectionMode &out) {
  if (!s) return false;
  if (strcmp(s, "PHYSICAL_BUTTON_DRIVEN") == 0) {
    out = InjectionMode::PhysicalButtonDriven;
    return true;
  }
  if (strcmp(s, "LOAD_LEVEL") == 0) {
    out = InjectionMode::LoadLevel;
    return true;
  }
  if (strcmp(s, "PERIODIC") == 0) {
    out = InjectionMode::Periodic;
    return true;
  }
  return false;
}

bool loadInjectionConfigFromLittleFS(InjectionConfigContext &out) {
  out = InjectionConfigContext::defaults();

  if (!LittleFS.begin()) {
    return false;
  }

  File f = LittleFS.open(INJECTION_SCHEDULE_JSON_PATH, "r");
  if (!f) {
    return false;
  }

  String raw = f.readString();
  f.close();

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, raw);
  if (err) {
    return false;
  }

  JsonObject sched = doc["injection_schedule"];
  if (sched.isNull()) {
    return false;
  }

  const char *modeStr = sched["mode"] | "";
  InjectionMode m;
  if (!parseModeString(modeStr, m)) {
    return false;
  }
  out.mode = m;
  out.tick_ms = sched["tick_ms"] | 10000u;
  if (out.tick_ms < 100u) {
    out.tick_ms = 100u;
  }
  out.load_level = sched["load_level"] | 1.0f;
  if (out.load_level < 0.0f) {
    out.load_level = 0.0f;
  }
  out.seed = sched["seed"] | 0u;
  out.loaded_from_fs = true;
  return true;
}

int computeStochasticInjectCount(float L) {
  if (L <= 0.0f) {
    return 0;
  }
  int base = (int)floorf(L);
  float frac = L - (float)base;
  int inject = base;
  if (frac > 0.0f) {
    long r = random(1000000L);
    if ((float)r < frac * 1000000.0f) {
      inject += 1;
    }
  }
  return inject;
}
