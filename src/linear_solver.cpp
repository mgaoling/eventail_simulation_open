#include "eventail_simulation/linear_solver.hpp"

namespace eventail_simulation {

LinearSolver::LinearSolver(const Matrix3d& K, const Matrix3d& K_inv, const ErrorMetric& metric_type,
                           const ErrorModel& model_type)
    : K_(K), K_inv_(K_inv), error_metric_type_(metric_type), error_model_type_(model_type) {
  reset();
}

void LinearSolver::inputGroundTruth(const Position& Pa, const Position& Pb, const Velocity& vc) {
  if (abs(Pa.x() + 1.0) > EPSILON6 || abs(Pb.x() - 1.0) > EPSILON6)
    EVENTAIL_WARN("The input ground truth does not match the two-point-two-plane parametrization!");

  Basis e1_gt, e2_gt, e3_gt;
  Velocity vl_gt;
  RotationMatrix Rcl_orthogonal_gt, Rcl_orthonormal_gt;
  vc_gt_ = vc;
  e1_gt = Pb - Pa;
  e2_gt = Pb.cross(Pa);
  e3_gt = e1_gt.cross(e2_gt);
  Rcl_orthogonal_gt.col(0) = e1_gt;
  Rcl_orthogonal_gt.col(1) = e2_gt;
  Rcl_orthogonal_gt.col(2) = e3_gt;
  Rcl_orthonormal_gt = turnRotationMatrixOrthonormal(Rcl_orthogonal_gt);
  vl_gt = Rcl_orthonormal_gt.transpose() * vc;
  vl_gt[0] = 0.0;  // ! the unobservable component due to aperture problem
  vc_gt_partial_ = Rcl_orthonormal_gt * vl_gt;
}

void LinearSolver::reset() {
  success_ = false;
  norm_e1_est_.setZero();
  norm_e2_est_.setZero();
  norm_e3_est_.setZero();
  ul_est_.setZero();
  vc_est_.setZero();
  cov_ = CovarianceMatrix::Zero(5, 5);
}

RotationMatrix LinearSolver::preRotation(const Measurements& measurements) {
  FiveEventMatrix data;
  for (int idx = 0; idx < measurements.first.size(); ++idx) {
    Eventd e = measurements.first[idx];
    Vector3d event_homo(e.x, e.y, 1.0);
    Vector3d unrotated_event_homo = K_ * axisAngleToRotationMatrix(measurements.second, e.t) * K_inv_ * event_homo;
    data(0, idx) = unrotated_event_homo.x() / unrotated_event_homo.z();
    data(1, idx) = unrotated_event_homo.y() / unrotated_event_homo.z();
  }
  FiveEventMatrix mean_data = data.colwise() - data.rowwise().mean();
  Eigen::JacobiSVD<Matrix2d> svd(mean_data * mean_data.transpose(), Eigen::ComputeFullV);
  RotationMatrix R = RotationMatrix::Identity();
  R.block<2, 2>(0, 0) = svd.matrixV();
  // ! make sure the Rotation Matrix follows the right hand rule
  if (abs(svd.matrixV().determinant() - 1) > EPSILON6) {
    R(0, 0) = -R(0, 0);
    R(1, 0) = -R(1, 0);
  }
  return R;
}

bool LinearSolver::linearSolve(const vector<Bearing>& fs, const vector<double>& ts) {
  MatrixXd A(ts.size(), 6);
  for (int i = 0; i < ts.size(); i++) {
    A.block<1, 3>(i, 0) = ts[i] * fs[i].transpose();
    A.block<1, 3>(i, 3) = -fs[i].transpose();
  }
  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullV);
  Vector3d a = svd.matrixV().col(5).head(3);
  Vector3d b = svd.matrixV().col(5).tail(3);
  a = a / b.norm();
  b = b / b.norm();
  ul_est_ << 0.0, (a - b.dot(a) * b).norm(), -b.dot(a);
  norm_e2_est_ = b;
  norm_e3_est_ = (a + norm_e2_est_ * ul_est_(2)) / (a + norm_e2_est_ * ul_est_(2)).norm();
  norm_e1_est_ = norm_e2_est_.cross(norm_e3_est_);

  return true;
}

bool LinearSolver::runSolver(const Measurements& measurements, double& duration, bool standardization) {
  if (measurements.first.size() < 5) {
    EVENTAIL_WARN("The input measurements shall be larger or equal to five events!");
    return false;
  }
  reset();

  // compose the bearing vectors to suit the linear solver
  vector<Bearing> bv_vec;
  vector<double> ts_vec;
  RotationMatrix R_pca = RotationMatrix::Identity();
  if (standardization) R_pca = preRotation(measurements);
  for (const auto& e : measurements.first) {
    Vector3d event_homo(e.x, e.y, 1.0);
    Bearing bv = R_pca * axisAngleToRotationMatrix(measurements.second, e.t) * K_inv_ * event_homo;
    bv.normalize();
    bv_vec.emplace_back(bv);
    ts_vec.push_back(e.t);
  }

  // execute the linear solver
  Timer clock;
  clock.start();
  if (!linearSolve(bv_vec, ts_vec)) return false;
  duration = clock.stop();

  // transform raw solutions into common geometric elements
  norm_e1_est_ = R_pca.transpose() * norm_e1_est_;
  norm_e2_est_ = R_pca.transpose() * norm_e2_est_;
  norm_e3_est_ = R_pca.transpose() * norm_e3_est_;
  vc_est_ = norm_e2_est_ * ul_est_(1) + norm_e3_est_ * ul_est_(2);
  success_ = true;
  return true;
}

bool LinearSolver::calculateCovariance(const Measurements& measurements) { return true; }

FullLinearSolution LinearSolver::getSolution() const {
  FullLinearSolution solution;
  if (!success_) {
    EVENTAIL_WARN("Solver has not produced reliable solution yet. Output empty solution instead!");
    return solution;
  }
  solution.first.col(0) = norm_e1_est_;
  solution.first.col(1) = norm_e2_est_;
  solution.first.col(2) = norm_e3_est_;
  solution.first.col(3) = ul_est_;
  solution.first.col(4) = vc_est_;
  solution.second = cov_;
  return solution;
}

// * Note that the solver only generates a partial estimation of velocity. Therefore, the comparison made here pertains
// * to the estimated velocity, which is normalized and relative to the camera frame.
double LinearSolver::getDirectionError() const {
  if (!success_) {
    EVENTAIL_WARN("Solver has not produced reliable solution yet. Output -1 instead!");
    return -1.0;
  }
  Velocity norm_2dof_vc_gt, norm_2dof_vc_est = vc_est_;
  if (error_model_type_ == ErrorModel::MotionAndGeometry)
    norm_2dof_vc_gt = vc_gt_partial_;
  else  // ! default choice (ErrorModel::MotionOnly)
    norm_2dof_vc_gt = (RotationMatrix::Identity() - norm_e1_est_ * norm_e1_est_.transpose()) * vc_gt_;
  norm_2dof_vc_gt.normalize();
  norm_2dof_vc_est.normalize();
  double theta = safe_acos_deg(norm_2dof_vc_gt.dot(norm_2dof_vc_est));
  return min(abs(theta), abs(180.0 - theta));  // ! to skip confirmation on solution duality
}

// ! Without resolving the ambiguity regarding the solution duality, it is advised against using the Euclidean Distance
// ! as the error function. In fact, during the simulation, there is a possibility of generating a landmark that falls
// ! outside the camera's field of view. This situation can pose challenges in accurately determining the correct one.
double LinearSolver::getEuclideanDistance() const {
  if (!success_) {
    EVENTAIL_WARN("Solver has not produced reliable solution yet. Output -1 instead!");
    return -1.0;
  }
  Velocity norm_2dof_vc_gt, norm_2dof_vc_est = vc_est_;
  if (error_model_type_ == ErrorModel::MotionAndGeometry)
    norm_2dof_vc_gt = vc_gt_partial_;
  else  // ! default choice (ErrorModel::MotionOnly)
    norm_2dof_vc_gt = (RotationMatrix::Identity() - norm_e1_est_ * norm_e1_est_.transpose()) * vc_gt_;
  norm_2dof_vc_gt.normalize();
  norm_2dof_vc_est.normalize();
  return (norm_2dof_vc_gt - norm_2dof_vc_est).squaredNorm();
}

double LinearSolver::getError() const {
  if (error_metric_type_ == ErrorMetric::EuclideanDistance)
    return getEuclideanDistance();
  else  // ! default choice (ErrorMetric::DirectionError)
    return getDirectionError();
}

}  // namespace eventail_simulation
