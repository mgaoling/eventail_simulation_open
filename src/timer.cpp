#include "eventail_simulation/timer.hpp"

namespace eventail_simulation {

Timer::Timer() : state_(ClockState::OFF), start_time_(Clock::now()), duration_(0.0) {}

void Timer::start() {
  if (state_ == ClockState::ON)
    EVENTAIL_WARN("The timer has not stopped yet! Cannot restart the clock.");
  else {
    state_ = ClockState::ON;
    start_time_ = Clock::now();
    duration_ = 0.0;
  }
}

double Timer::pulse() {
  if (state_ == ClockState::OFF) {
    EVENTAIL_WARN("The timer has already stopped! Return saved duration time instead.");
    return duration_;
  } else
    return std::chrono::duration_cast<Microsecond>(Clock::now() - start_time_).count();
}

double Timer::stop() {
  if (state_ == ClockState::OFF) {
    EVENTAIL_WARN("The timer has already stopped! Return saved duration time instead.");
    return duration_;
  } else {
    state_ = ClockState::OFF;
    duration_ = std::chrono::duration_cast<Microsecond>(Clock::now() - start_time_).count();
    return duration_;
  }
};

double Timer::getDuration() {
  if (state_ == ClockState::ON) {
    EVENTAIL_WARN("The timer has not stopped yet! Return zero as duration time instead.");
    return 0.0;
  } else
    return duration_;
}

}  // namespace eventail_simulation
