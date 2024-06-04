#pragma once

#include <algorithm>
#include <random>

#include "types.hpp"

namespace eventail_simulation {

class Random {
 public:
  using Ptr = shared_ptr<Random>;
  using UniformInt = std::uniform_int_distribution<int>;
  using UniformReal = std::uniform_real_distribution<double>;
  using GaussianReal = std::normal_distribution<double>;
  Random();

  // Basic Utility
  inline bool flipCoin() { return binary_scalar_(det_gen_) != 0; }
  double sampleUniformScalar(double min, double max);
  double sampleGaussianScalar(double mean, double std_dev);
  Vector2d sample2dVector(double min_magnitude, double max_magnitude);
  Vector3d sample3dVector(double min_magnitude, double max_magnitude);

  // Detailed Functions
  double sampleTimestamp(const double_pair& time_window);
  double sampleDepth(const double_pair& depth_range);
  Velocity sampleLinearVelocity(const double_pair& linear_velocity_range);
  Velocity sampleAngularVelocity(const double_pair& angular_velocity_range);
  vector<int> sampleShuffledRange(int N);

  // Line Utility
  Feature sample2dPoint(int img_width, int img_height);
  Line2D sample2dLine(int img_width, int img_height);
  Line2DPair sample2dLinePair(int img_width, int img_height);

  // Noise Utility
  Eventd sampleRandomEvent(int img_width, int img_height, const double_pair& time_window);
  Vector2d sampleUniformPixelNoise(double noise_magnitude);
  Vector2d sampleGaussianPixelNoise(double noise_std_dev);
  double sampleUniformTimestampJitter(double noise_magnitude);
  double sampleGaussianTimestampJitter(double noise_std_dev);
  Vector3d sampleUniformAngularVelocityDisturbance(double noise_magnitude);
  Vector3d sampleGaussianAngularVelocityDisturbance(double noise_std_dev);
  inline void shuffleEventSet(EventSet& events) { std::shuffle(begin(events), end(events), det_gen_); }

 private:
  std::mt19937 det_gen_;
  UniformInt binary_scalar_;
  UniformReal uniform_scalar_01d_;  // uniformly distributed on the interval [0, 1)
  GaussianReal gaussian_scalar_;
};

}  // namespace eventail_simulation
