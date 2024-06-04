#pragma once

#include <chrono>

#include "types.hpp"

namespace eventail_simulation {

class Timer {
 public:
  explicit Timer();

  void start();
  double pulse();  // Return current duration time and the timer keeps going.
  double stop();   // Return total duration time and the timer stops here.
  double getDuration();

 private:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Microsecond = std::chrono::microseconds;
  enum class ClockState { ON, OFF };

  ClockState state_;
  TimePoint start_time_;
  double duration_;
};

}  // namespace eventail_simulation
