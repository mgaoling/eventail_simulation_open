#pragma once

#include "types.hpp"

namespace eventail_simulation {

inline void glogVector(string description, vector<double> vec) {
  std::stringstream log;
  for (auto value : vec) log << value << ", ";
  std::string msg = '[' + log.str().substr(0, log.str().size() - 2) + ']';

  // ! Each message in GLOG only allows a maximum character size of 30,000.
  constexpr size_t max_log_size = 29500;

  LOG(INFO) << description;
  for (size_t idx = 0; idx * max_log_size < msg.size(); ++idx) {
    size_t start_pos = idx * max_log_size;
    size_t len = max_log_size;
    if (start_pos + max_log_size > msg.size()) len = msg.size() - start_pos;
    LOG(INFO) << msg.substr(start_pos, len);
  }
}

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

inline double safe_acos_rad(double x) { return abs(x) >= (1.0 - EPSILON9) ? 0.0 : acos(x); }

inline double safe_acos_deg(double x) { return abs(x) >= (1.0 - EPSILON9) ? 0.0 : rad2deg(acos(x)); }

inline Velocity svd(const MatrixXd& mtx) {
  Eigen::JacobiSVD<MatrixXd> svd(mtx, Eigen::ComputeFullV);
  return svd.matrixV().col(2);
}

inline Velocity standardizedSvd(const MatrixXd& mtx) {
  VectorXd mean, std_dev;
  mean = mtx.colwise().mean();
  std_dev = ((mtx.rowwise() - mean.transpose()).array().square().colwise().sum() / (mtx.rows() - 1)).sqrt();
  MatrixXd M = MatrixXd::Identity(4, 4);
  M.block<3, 3>(0, 0) = std_dev.asDiagonal().inverse();
  M.block<1, 3>(3, 0) = -mean.cwiseQuotient(std_dev);
  MatrixXd extended_mtx(mtx.rows(), 4);
  extended_mtx << mtx, MatrixXd::Ones(mtx.rows(), 1);
  Eigen::JacobiSVD<MatrixXd> svd(extended_mtx * M, Eigen::ComputeFullV);
  return M * svd.matrixV().col(3);
}

inline RotationMatrix axisAngleToRotationMatrix(const Velocity& angular, double t) {
  double angle = angular.norm() * t;
  Vector3d axis = angular;
  axis.normalize();
  return AngleAxisd(angle, axis).toRotationMatrix();
}

inline RotationMatrix turnRotationMatrixOrthonormal(const RotationMatrix& R_orthogonal) {
  RotationMatrix R_orthonormal = R_orthogonal;
  R_orthonormal.col(0).normalize();
  R_orthonormal.col(1).normalize();
  R_orthonormal.col(2).normalize();
  return R_orthonormal;
}

// Ref: https://stackoverflow.com/questions/6408670/line-of-intersection-between-two-planes
inline bool solveIntersectionOfPlanes(const Position& point1, const Vector3d& normal1, const Vector3d& point2,
                                      const Vector3d& normal2, Line3D& est_line) {
  Vector3d line_direction = normal1.cross(normal2);
  if (line_direction.squaredNorm() < EPSILON6) return false;  // make sure plane is not parallel to each other
  if (abs(line_direction.x()) < EPSILON6) return false;       // ! make sure plane is not parallel to yz plane
  double plane1_direction = -normal1.dot(point1) / normal1.norm();
  double plane2_direction = -normal2.dot(point2) / normal2.norm();
  Vector3d point_on_line =
      (line_direction.cross(normal2) * plane1_direction + normal1.cross(line_direction) * plane2_direction) /
      line_direction.squaredNorm();
  double ratio1 = (-1.0 - point_on_line.x()) / line_direction.x();
  Landmark normalized_landmark1 = point_on_line + ratio1 * line_direction;
  double ratio2 = (1.0 - point_on_line.x()) / line_direction.x();
  Landmark normalized_landmark2 = point_on_line + ratio2 * line_direction;
  est_line = make_pair(normalized_landmark1, normalized_landmark2);
  return true;
}

}  // namespace eventail_simulation
