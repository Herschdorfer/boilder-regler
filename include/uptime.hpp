#pragma once

#include <chrono>

class Uptime {
public:
  Uptime();
  uint32_t getUptimeInSeconds() const;

private:
  std::chrono::steady_clock::time_point startTime;
};
