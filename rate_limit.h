#ifndef RATE_LIMIT_H_
#define RATE_LIMIT_H_

#include <cstdint>

/**
 * @brief Used to run a loop at a constant rate.
 */
class RateLimit {
 public: 
  /**
   * @brief The period of the loop, in ms.
   */
  RateLimit(uint16_t period);

  /**
   * @brief Run this once per loop to delay for
   *   the proper amount of time.
   */
  void Limit();

 private:
  /// The target period, in us.
  uint32_t period_;
  /// The timestamp of the last call to Limit().
  uint32_t previous_call_time_ = 0;
};

#endif  // RATE_LIMIT_H_
