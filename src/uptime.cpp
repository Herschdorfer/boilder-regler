
#include <chrono>
#include <cstdint>
#include <iostream>

#include "uptime.hpp"

class Uptime;

Uptime::Uptime() { startTime = std::chrono::steady_clock::now(); }

std::uint32_t Uptime::getUptimeInSeconds() const {
  auto current_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(
      current_time - startTime);
  return static_cast<uint32_t>(duration.count());
}
