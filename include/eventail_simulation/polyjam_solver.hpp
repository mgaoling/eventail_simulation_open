#pragma once

#include <limits>

#include "geometry.hpp"
#include "timer.hpp"
#include "types.hpp"

namespace eventail_simulation {

class PolyjamSolver {
 public:
  using Ptr = shared_ptr<PolyjamSolver>;
  PolyjamSolver() = default;
  PolyjamSolver(const Matrix3d& K, const Matrix3d& K_inv, const ErrorMetric& metric_type, const ErrorModel& model_type);

  void inputGroundTruth(const Position& Pa, const Position& Pb, const Velocity& vc);
  bool runSolver(const Measurements& measurements, double& duration, bool standardization = false);
  bool calculateCovariance(const Measurements& measurements);
  FullPolyjamSolution getSolution() const;
  double getError() const;

 private:
  Matrix3d K_, K_inv_;
  ErrorMetric error_metric_type_;
  ErrorModel error_model_type_;

  bool success_;
  double vy_est_, vz_est_, ya_est_, za_est_, zb_est_, yb_est_;  // ! in this order
  Position Pa_est_, Pb_est_, Pa_gt_, Pb_gt_;
  Basis e1_est_, e2_est_, e3_est_, e1_gt_, e2_gt_, e3_gt_;
  Velocity norm_vc_est_, vl_est_, vc_gt_, vl_gt_, vc_gt_partial_;
  RotationMatrix Rcl_orthogonal_est_, Rcl_orthogonal_gt_, Rcl_orthonormal_gt_;
  CovarianceMatrix cov_;

  void reset();
  RotationMatrix preRotation(const Measurements& measurements);
  void initRow(Eigen::MatrixXd& M2, const Eigen::MatrixXd& M1, int row2, int row1, const int* cols2, const int* cols1,
               size_t numberCols);
  bool polyjamSolve(const std::vector<Bearing>& fs, const std::vector<double>& ts);
  double getDirectionError() const;
  double getEuclideanDistance() const;
};

}  // namespace eventail_simulation
