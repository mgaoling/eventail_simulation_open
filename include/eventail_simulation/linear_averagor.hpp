#pragma once

#include "geometry.hpp"
#include "types.hpp"

namespace eventail_simulation {

class LinearAveragor {
 public:
  using Ptr = shared_ptr<LinearAveragor>;
  LinearAveragor() = default;
  LinearAveragor(const double uncertainty_level, const ErrorMetric& metric_type, const ErrorModel& model_type,
                 const SvdFunction& svd_type);

  void reset();
  void inputGroundTruth(const Velocity& vc);
  void addData(const FullLinearSolution& solution);
  bool runAveragor(bool use_uncertainty = false);
  Velocity getEstimation() const;
  double getError() const;

 private:
  ErrorMetric error_metric_type_;
  ErrorModel error_model_type_;
  SvdFunction svd_type_;
  double uncertainty_level_;

  bool success_;
  Velocity vc_est_, vc_gt_;
  vector<FullLinearSolution> solutions_;

  double getDirectionError() const;
  double getEuclideanDistance() const;
};

}  // namespace eventail_simulation
