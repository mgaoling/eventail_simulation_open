#include "eventail_simulation/polyjam_averagor.hpp"

namespace eventail_simulation {

PolyjamAveragor::PolyjamAveragor(const double uncertainty_level, const ErrorMetric& metric_type,
                                 const ErrorModel& model_type, const SvdFunction& svd_type)
    : uncertainty_level_(uncertainty_level),
      error_metric_type_(metric_type),
      error_model_type_(model_type),
      svd_type_(svd_type){};

void PolyjamAveragor::reset() {
  success_ = false;
  vc_gt_.setZero();
  vc_est_.setZero();
  solutions_.clear();
}

void PolyjamAveragor::inputGroundTruth(const Velocity& vc) { vc_gt_ = vc; }

void PolyjamAveragor::addData(const FullPolyjamSolution& solution) {
  if (solution.second.determinant() >= uncertainty_level_) solutions_.emplace_back(solution);
}

bool PolyjamAveragor::runAveragor(bool use_uncertainty) {
  if (solutions_.size() < 2) {
    EVENTAIL_WARN("Less than two eventails have been added to the Averagor! Rank deficient!");
    vc_est_ = Velocity::Zero();
    return false;
  }

  Matrix3d U = Matrix3d::Zero();
  MatrixXd V_inv = MatrixXd::Zero(solutions_.size(), solutions_.size());
  MatrixXd W = MatrixXd::Zero(3, solutions_.size());
  for (size_t idx = 0; idx < solutions_.size(); ++idx) {
    Position Pa = solutions_[idx].first.col(0);
    Position Pb = solutions_[idx].first.col(1);
    Velocity vl = solutions_[idx].first.col(2);
    Basis e1 = Pb - Pa;
    Basis e2 = Pb.cross(Pa);
    Basis e3 = e1.cross(e2);
    double e2_sq_norm = e2.squaredNorm();
    double e3_sq_norm = e3.squaredNorm();
    U += e2 * e2.transpose() / pow(e2_sq_norm, 2) + e3 * e3.transpose() / pow(e3_sq_norm, 2);
    V_inv(idx, idx) = 1.0 / vl.squaredNorm();
    W.col(idx) = -vl[1] * e2 / e2_sq_norm - vl[2] * e3 / e3_sq_norm;
  }
  Matrix3d D = U - W * V_inv * W.transpose();

  if (D.rows() == 2 || svd_type_ == SvdFunction::SVD)
    vc_est_ = svd(D);
  else  // ! default choice (SvdFunction::PCA)
    vc_est_ = standardizedSvd(D);
  success_ = true;
  return true;
}

Velocity PolyjamAveragor::getEstimation() const {
  if (!success_) {
    EVENTAIL_WARN("Averagor has not produced reliable solution yet. Output zero velocity instead!");
    return Velocity::Zero();
  } else
    return vc_est_;
}

double PolyjamAveragor::getDirectionError() const {
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

double PolyjamAveragor::getEuclideanDistance() const {
  if (!success_) {
    EVENTAIL_WARN("Averagor has not produced reliable solution yet. Output -1 instead!");
    return -1.0;
  }
  Velocity norm_vc_gt = vc_gt_, norm_vc_est = vc_est_;
  norm_vc_gt.normalize();
  norm_vc_est.normalize();
  return (norm_vc_gt - norm_vc_est).squaredNorm();
}

double PolyjamAveragor::getError() const {
  if (error_metric_type_ == ErrorMetric::EuclideanDistance)
    return getEuclideanDistance();
  else  // ! default choice (ErrorMetric::DirectionError)
    return getDirectionError();
}

}  // namespace eventail_simulation
