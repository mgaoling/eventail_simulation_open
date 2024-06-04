#pragma once

#include "geometry.hpp"
#include "timer.hpp"
#include "types.hpp"

namespace eventail_simulation {

class LinearSolver {
 public:
  using Ptr = shared_ptr<LinearSolver>;
  LinearSolver() = default;
  LinearSolver(const Matrix3d& K, const Matrix3d& K_inv, const ErrorMetric& metric_type, const ErrorModel& model_type);

  void inputGroundTruth(const Position& Pa, const Position& Pb, const Velocity& vc);
  bool runSolver(const Measurements& measurements, double& duration, bool standardization = false);
  bool calculateCovariance(const Measurements& measurements);
  FullLinearSolution getSolution() const;
  double getError() const;

 private:
  Matrix3d K_, K_inv_;
  ErrorMetric error_metric_type_;
  ErrorModel error_model_type_;

  bool success_;
  Basis norm_e1_est_, norm_e2_est_, norm_e3_est_;
  Velocity ul_est_, vc_est_, vc_gt_, vc_gt_partial_;
  CovarianceMatrix cov_;

  void reset();
  RotationMatrix preRotation(const Measurements& measurements);
  bool linearSolve(const vector<Bearing>& fs, const vector<double>& ts);
  double getDirectionError() const;
  double getEuclideanDistance() const;
};

}  // namespace eventail_simulation
