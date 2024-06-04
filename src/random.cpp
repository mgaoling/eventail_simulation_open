#include "eventail_simulation/random.hpp"

DEFINE_double(SEED, 42, "to initialize the pseudorandom number generator");

namespace eventail_simulation {

Random::Random()
    : det_gen_(FLAGS_SEED), binary_scalar_(0, 1), uniform_scalar_01d_(0.0, 1.0), gaussian_scalar_(0.0, 1.0) {
  LOG(INFO) << "Random Seed: " << FLAGS_SEED;
}

double Random::sampleUniformScalar(double min, double max) {
  return min + (max - min) * uniform_scalar_01d_(det_gen_);
};

double Random::sampleGaussianScalar(double mean, double std_dev) {
  gaussian_scalar_.param(GaussianReal::param_type(mean, std_dev));
  return gaussian_scalar_(det_gen_);
};

Vector2d Random::sample2dVector(double min_magnitude, double max_magnitude) {
  double theta = sampleUniformScalar(0.0, 2 * M_PI);
  Vector2d unit_vec = Vector2d(cos(theta), sin(theta));
  return sampleUniformScalar(min_magnitude, max_magnitude) * unit_vec;
}

// Ref: https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
Vector3d Random::sample3dVector(double min_magnitude, double max_magnitude) {
  double z = sampleUniformScalar(-1.0, 1.0);
  double theta = sampleUniformScalar(0.0, 2 * M_PI);
  double r = sqrt(1 - z * z);
  double x = r * cos(theta);
  double y = r * sin(theta);
  Vector3d unit_vec = Vector3d(x, y, z);
  return sampleUniformScalar(min_magnitude, max_magnitude) * unit_vec;
}

double Random::sampleTimestamp(const double_pair& time_window) {
  return sampleUniformScalar(time_window.first, time_window.second);
}

double Random::sampleDepth(const double_pair& depth_range) {
  return sampleUniformScalar(depth_range.first, depth_range.second);
}

Velocity Random::sampleLinearVelocity(const double_pair& linear_velocity_range) {
  return sample3dVector(linear_velocity_range.first, linear_velocity_range.second);
}

Velocity Random::sampleAngularVelocity(const double_pair& angular_velocity_range) {
  return sample3dVector(angular_velocity_range.first, angular_velocity_range.second);
}

// * Output a shuffled vector containing index values from 0 to N-1.
vector<int> Random::sampleShuffledRange(int N) {
  vector<int> idx_vec;
  for (int idx = 0; idx < N; ++idx) idx_vec.emplace_back(idx);
  std::shuffle(begin(idx_vec), end(idx_vec), det_gen_);
  return idx_vec;
}

Feature Random::sample2dPoint(int img_width, int img_height) {
  return Feature(round(sampleUniformScalar(0, img_width - 1)), round(sampleUniformScalar(0, img_height - 1)));
}

Line2D Random::sample2dLine(int img_width, int img_height) {
  return make_pair(sample2dPoint(img_width, img_height), sample2dPoint(img_width, img_height));
}

Line2DPair Random::sample2dLinePair(int img_width, int img_height) {
  return make_pair(sample2dLine(img_width, img_height), sample2dLine(img_width, img_height));
}

Eventd Random::sampleRandomEvent(int img_width, int img_height, const double_pair& time_window) {
  Feature pt = sample2dPoint(img_width, img_height);
  return Eventd(pt[0], pt[1], sampleTimestamp(time_window), 0);
}

Vector2d Random::sampleUniformPixelNoise(double noise_magnitude) {
  return sample2dVector(noise_magnitude, noise_magnitude);
}

Vector2d Random::sampleGaussianPixelNoise(double noise_std_dev) {
  return Feature(sampleGaussianScalar(0, noise_std_dev), sampleGaussianScalar(0, noise_std_dev));
}

double Random::sampleUniformTimestampJitter(double noise_magnitude) {
  return sampleUniformScalar(-noise_magnitude, noise_magnitude);
}

double Random::sampleGaussianTimestampJitter(double noise_std_dev) { return sampleGaussianScalar(0, noise_std_dev); }

Vector3d Random::sampleUniformAngularVelocityDisturbance(double noise_magnitude) {
  return sample3dVector(noise_magnitude, noise_magnitude);
}

Vector3d Random::sampleGaussianAngularVelocityDisturbance(double noise_std_dev) {
  return Vector3d(sampleGaussianScalar(0, noise_std_dev), sampleGaussianScalar(0, noise_std_dev),
                  sampleGaussianScalar(0, noise_std_dev));
}

}  // namespace eventail_simulation
