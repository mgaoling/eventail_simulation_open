#include "eventail_simulation/linear_averagor.hpp"

namespace eventail_simulation {

LinearAveragor::LinearAveragor(const double uncertainty_level, const ErrorMetric& metric_type,
                               const ErrorModel& model_type, const SvdFunction& svd_type)
    : uncertainty_level_(uncertainty_level),
      error_metric_type_(metric_type),
      error_model_type_(model_type),
      svd_type_(svd_type){};

void LinearAveragor::reset() {
  success_ = false;
  vc_gt_.setZero();
  vc_est_.setZero();
  solutions_.clear();
}

void LinearAveragor::inputGroundTruth(const Velocity& vc) { vc_gt_ = vc; }

void LinearAveragor::addData(const FullLinearSolution& solution) {
  if (solution.second.determinant() >= uncertainty_level_) solutions_.emplace_back(solution);
}

bool LinearAveragor::runAveragor(bool use_uncertainty) {
  if (solutions_.size() < 2) {
    EVENTAIL_WARN("Less than two eventails have been added to the Averagor! Rank deficient!");
    vc_est_ = Velocity::Zero();
    return false;
  }

  MatrixXd D = MatrixXd::Zero(solutions_.size(), 3);
  for (size_t idx = 0; idx < solutions_.size(); ++idx) {
    Basis e2 = solutions_[idx].first.col(1);
    Basis e3 = solutions_[idx].first.col(2);
    double uy = solutions_[idx].first.col(3)[1];
    double uz = solutions_[idx].first.col(3)[2];
    D.row(idx) = uz * e2.transpose() - uy * e3.transpose();
  }

  if (D.rows() == 2 || svd_type_ == SvdFunction::SVD)
    vc_est_ = svd(D);
  else  // ! default choice (SvdFunction::PCA)
    vc_est_ = standardizedSvd(D);
  success_ = true;
  return true;
}

Velocity LinearAveragor::getEstimation() const {
  if (!success_) {
    EVENTAIL_WARN("Averagor has not produced reliable solution yet. Output zero velocity instead!");
    return Velocity::Zero();
  } else
    return vc_est_;
}

double LinearAveragor::getDirectionError() const {
  if (!success_) {
    EVENTAIL_WARN("Averagor has not produced reliable solution yet. Output -1 instead!");
    return -1.0;
  }

  Velocity norm_vc_gt = vc_gt_, norm_vc_est = vc_est_;
  norm_vc_gt.normalize();
  norm_vc_est.normalize();
  double theta = safe_acos_deg(norm_vc_gt.dot(norm_vc_est));
  return min(abs(theta), abs(180.0 - theta));  // ! to skip confirmation on solution duality
}

double LinearAveragor::getEuclideanDistance() const {
  if (!success_) {
    EVENTAIL_WARN("Averagor has not produced reliable solution yet. Output -1 instead!");
    return -1.0;
  }
  Velocity norm_vc_gt = vc_gt_, norm_vc_est = vc_est_;
  norm_vc_gt.normalize();
  norm_vc_est.normalize();
  return (norm_vc_gt - norm_vc_est).squaredNorm();
}

double LinearAveragor::getError() const {
  if (error_metric_type_ == ErrorMetric::EuclideanDistance)
    return getEuclideanDistance();
  else  // ! default choice (ErrorMetric::DirectionError)
    return getDirectionError();
}

}  // namespace eventail_simulation
