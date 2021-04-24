#include "rate_limit.h"

#include <Arduino.h>

RateLimit::RateLimit(uint16_t period) : period_(static_cast<uint32_t>(period) * 1000) {}

void RateLimit::Limit() {
  const uint32_t kCurrentTime = micros();
  const uint32_t kElapsed = kCurrentTime - previous_call_time_;
  previous_call_time_ = kCurrentTime;

  if (kElapsed < period_) {
    // Wait the remaining time.
    delayMicroseconds(period_ - kElapsed);
  }
}
