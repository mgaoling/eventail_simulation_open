#pragma once

#include "geometry.hpp"
#include "linear_averagor.hpp"
#include "linear_solver.hpp"
#include "polyjam_averagor.hpp"
#include "polyjam_solver.hpp"
#include "random.hpp"
#include "timer.hpp"
#include "types.hpp"

namespace eventail_simulation {

class Simulator {
 public:
  using Ptr = shared_ptr<Simulator>;
  Simulator();

  // getter functions
  inline int getSceneNum() const { return scene_num_; }
  inline int getNoiseNum() const { return noise_num_; }
  inline vector<double> getNoiseLevels() { return noise_levels_; }

  // sample functions
  Velocity sampleLinearVelocity();
  Velocity sampleAngularVelocity();
  Line3D sampleLine(const Velocity& linear, const Velocity& angular);
  EventSet sampleRandomEvents(int N);
  EventSet sampleEvents(int N, const Velocity& linear, const Velocity& angular, const Line3D& line);
  EventSet sampleFiveEvents(const Velocity& linear, const Velocity& angular, const Line3D& line);
  EventSet sampleTenEvents(const Velocity& linear, const Velocity& angular, const Line3D& line);
  Measurements sampleNoisyMeasurements(const EventSet& events, const Velocity& angular, double noise_level);

  // estimate functions
  void setupGroundTruth(const Velocity& linear, const Line3D& line);
  bool linearSolve(const Measurements& measurements, double& duration, double& error, bool uncertainty = false,
                   bool standardization = false);
  bool polyjamSolve(const Measurements& measurements, double& duration, double& error, bool uncertainty = false,
                    bool standardization = false);
  bool linearAverage(Velocity& linear_est, double& error, bool reset = true);
  bool polyjamAverage(Velocity& linear_est, double& error, bool reset = true);

 private:
  Random generator_;
  LinearSolver linear_solver_;
  PolyjamSolver polyjam_solver_;
  LinearAveragor linear_averagor_;
  PolyjamAveragor polyjam_averagor_;

  // High-level Configuration
  int scene_num_;
  int noise_num_;
  double_pair time_window_;
  ErrorMetric error_metric_type_;
  ErrorModel error_model_type_;
  SvdFunction svd_type_;

  // Camera Intrinsic Setup
  int img_width_;
  int img_height_;
  Matrix3d K_, K_inv_;

  // Scene Generation Setup
  LineModel line_type_;
  double_pair depth_range_;
  double_pair lin_vel_range_;
  double_pair ang_vel_range_;

  // Noise Simulation Setup
  EventSampleStrategy event_sample_type_;
  NoiseModel noise_type_;
  NoiseDistribution distribution_type_;
  vector<double> noise_levels_;

  // High-level Sampling Schemes
  bool isPointOnImage(Vector3d homo_pt) const;
  bool isLineInFov(const Line3D& l, const Velocity& linear, const Velocity& angular);
  Line3D sampleLineBySceneToCamera(const Velocity& linear, const Velocity& angular);
  Line3D sampleLineByCameraToScene(const Velocity& linear, const Velocity& angular);
  EventSet sampleEventsByRandom(int N, const Velocity& linear, const Velocity& angular, const Line3D& line);
  EventSet sampleEventsBySpatial(int N, const Velocity& linear, const Velocity& angular, const Line3D& line);
  EventSet sampleEventsByTemporal(int N, const Velocity& linear, const Velocity& angular, const Line3D& line);
  EventSet sampleEventsBySpatiotemporal(int N, const Velocity& linear, const Velocity& angular, const Line3D& line);
  EventSet addPixelNoise(const EventSet& events, double noise_level);                   // Uniformly-distributed Noises.
  EventSet addTimestampJitter(const EventSet& events, double noise_level);              // Zero-Mean Gaussian Noises.
  Velocity addAngularVelocityDisturbance(const Velocity& angular, double noise_level);  // Uniformly-distributed Noises.
};

}  // namespace eventail_simulation
