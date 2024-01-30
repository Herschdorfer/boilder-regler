#pragma once

#include <chrono>

/**
 * @brief Class to keep track of uptime
 *
 */
class Uptime {
public:
  /**
   * @brief Construct a new Uptime object
   *
   */
  Uptime();

  /**
   * @brief Destroy the Uptime object
   *
   */
  ~Uptime();

  /**
   * @brief Get the Uptime In Seconds object
   *
   * @return std::uint32_t
   */
  uint32_t getUptimeInSeconds() const;

private:
  /**
   * @brief Start time of the uptime
   *
   */
  std::chrono::steady_clock::time_point startTime;
};
