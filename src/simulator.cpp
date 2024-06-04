#include "eventail_simulation/simulator.hpp"

DEFINE_uint32(SCENE_SETUP_NUM, 100, "number of different scene configuration");
DEFINE_uint32(NOISE_SETUP_NUM, 100, "number of different noise configuration, within each scene configuration");
DEFINE_double(TIME_INTERVAL, 0.5, "interval length of the time window [s]");
DEFINE_uint32(ERROR_METRIC_TYPE, 0, "(0) Direction Error, (1) Euclidean Distance");
DEFINE_uint32(ERROR_MODEL_TYPE, 0, "(0) Motion Only (conservative), (1) Motion and Geometry (radical)");

DEFINE_uint32(IMAGE_WIDTH, 640, "offsets of the principal point, from the top-left corner of the image frame");
DEFINE_uint32(IMAGE_HEIGHT, 480, "offsets of the principal point, from the top-left corner of the image frame");
DEFINE_uint32(FOCAL_LENGTH, 320, "pixel focal length of the camera");

DEFINE_uint32(LINE_TYPE, 1, "(0) scene to camera, (1) camera to scene");
DEFINE_double(MIN_DEPTH, 3.0, "minimum depth of feature points, only used under the (1) scene to camear setup [m]");
DEFINE_double(MAX_DEPTH, 5.0, "maximum depth of feature points, only used under the (1) scene to camear setup [m]");
DEFINE_double(MIN_LINEAR_VELOCITY_MAGNITUDE, 1.0, "minimum magnitude of the camera linear velocity [m/s]");
DEFINE_double(MAX_LINEAR_VELOCITY_MAGNITUDE, 1.0, "maximum magnitude of the camera linear velocity [m/s]");
DEFINE_double(MIN_ANGULAR_VELOCITY_MAGNITUDE, 90.0, "minimum magnitude of the camera angular velocity [deg/s]");
DEFINE_double(MAX_ANGULAR_VELOCITY_MAGNITUDE, 90.0, "maximum magnitude of the camera angular velocity [deg/s]");
DEFINE_uint32(EVENT_SAMPLE_TYPE, 3, "(0) random, (1) spatial, (2) temporal, (3) spatiotemporal");

DEFINE_uint32(
    NOISE_TYPE, 0,
    "(0) noise free, (1) pixel noise [pix], (2) timestamp jitter [s], (3) angular velocity disturbance [deg/s]");
DEFINE_uint32(NOISE_DISTRIBUTION, 0, "(0) Uniform Distribution, (1) Gaussian Distribution");
DEFINE_double(MIN_NOISE_MAGNITUDE, 0.0, "minimum magnitude of the additive noise, according to noise type");
DEFINE_double(MAX_NOISE_MAGNITUDE, 1.0, "maximum magnitude of the additive noise, according to noise type");
DEFINE_double(NOISE_STEP_SIZE, 0.1, "the increments between noise value");

DEFINE_uint32(SVD_TYPE, 0, "(0) SVD, (1) SVD with standardization");
DEFINE_double(UNCERTAINTY_THRESHOLD, 0.0, "uncertainty threshold to rule out uncertain manifold estimation");

namespace eventail_simulation {

Simulator::Simulator() {
  // High-level Configuration
  scene_num_ = FLAGS_SCENE_SETUP_NUM;
  noise_num_ = FLAGS_NOISE_SETUP_NUM;
  time_window_ = make_pair(-FLAGS_TIME_INTERVAL / 2, FLAGS_TIME_INTERVAL / 2);
  LOG(INFO) << "Simulation Iteration: " << scene_num_ << "x" << noise_num_;
  LOG(INFO) << "Time Window: [" << time_window_.first << ", " << time_window_.second << "]";
  switch (FLAGS_ERROR_METRIC_TYPE) {
    case 0:
      error_metric_type_ = ErrorMetric::DirectionError;
      LOG(INFO) << "Error Metric Type: (0) Direction Error";
      break;
    case 1:
      error_metric_type_ = ErrorMetric::EuclideanDistance;
      LOG(INFO) << "Error Metric Type: (1) Euclidean Distance";
      break;
    default:
      EVENTAIL_WARN("Unknown Error Metric Type. Go with '(0) Direction Error' instead.");
      error_metric_type_ = ErrorMetric::DirectionError;
      LOG(INFO) << "Error Metric Type: (0) Direction Error";
      break;
  }
  switch (FLAGS_ERROR_MODEL_TYPE) {
    case 0:
      error_model_type_ = ErrorModel::MotionOnly;
      LOG(INFO) << "Error Model Type: (0) Motion Only (conservative)";
      break;
    case 1:
      error_model_type_ = ErrorModel::MotionAndGeometry;
      LOG(INFO) << "Error Model Type: (1) Motion and Geometry (radical)";
      break;
    default:
      EVENTAIL_WARN("Unknown Error Model Type. Go with '(0) Motion Only (conservative)' instead.");
      error_model_type_ = ErrorModel::MotionOnly;
      LOG(INFO) << "Error Model Type: (0) Motion Only (conservative)";
      break;
  }

  // Camera Intrinsic Setup
  img_width_ = FLAGS_IMAGE_WIDTH;
  img_height_ = FLAGS_IMAGE_HEIGHT;
  K_ << FLAGS_FOCAL_LENGTH, 0, img_width_ / 2, 0, FLAGS_FOCAL_LENGTH, img_height_ / 2, 0, 0, 1;
  K_inv_ = K_.inverse();
  LOG(INFO) << "Camera Intrinsic Matrix: \n" << K_;
  linear_solver_ = LinearSolver(K_, K_inv_, error_metric_type_, error_model_type_);
  polyjam_solver_ = PolyjamSolver(K_, K_inv_, error_metric_type_, error_model_type_);

  // Scene Generation Setup
  switch (FLAGS_LINE_TYPE) {
    case 0:
      line_type_ = LineModel::Scene2Camera;
      depth_range_ = make_pair(FLAGS_MIN_DEPTH, FLAGS_MAX_DEPTH);
      LOG(INFO) << "Line Type: (0) scene to camera";
      LOG(INFO) << "Feature Depth Range: [" << depth_range_.first << ", " << depth_range_.second << "]";
      break;
    case 1:
      line_type_ = LineModel::Camera2Scene;
      LOG(INFO) << "Line Type: (1) camera to scene";
      break;
    default:
      EVENTAIL_WARN("Unknown Line Type. Go with '(1) camera to scene' instead.");
      line_type_ = LineModel::Camera2Scene;
      LOG(INFO) << "Line Type: (1) camera to scene";
      break;
  }
  lin_vel_range_ = make_pair(FLAGS_MIN_LINEAR_VELOCITY_MAGNITUDE, FLAGS_MAX_LINEAR_VELOCITY_MAGNITUDE);
  ang_vel_range_ =
      make_pair(deg2rad(FLAGS_MIN_ANGULAR_VELOCITY_MAGNITUDE), deg2rad(FLAGS_MAX_ANGULAR_VELOCITY_MAGNITUDE));
  LOG(INFO) << "Linear Velocity Range: [" << lin_vel_range_.first << ", " << lin_vel_range_.second << "]";
  LOG(INFO) << "Angular Velocity Range: [" << ang_vel_range_.first << ", " << ang_vel_range_.second << "]";

  // Noise Simulation Setup
  switch (FLAGS_EVENT_SAMPLE_TYPE) {
    case 0:
      event_sample_type_ = EventSampleStrategy::Random;
      LOG(INFO) << "Event Sample Type: (0) random";
      break;
    case 1:
      event_sample_type_ = EventSampleStrategy::Spatial;
      LOG(INFO) << "Event Sample Type: (1) spatial";
      break;
    case 2:
      event_sample_type_ = EventSampleStrategy::Temporal;
      LOG(INFO) << "Event Sample Type: (2) temporal";
      break;
    case 3:
      event_sample_type_ = EventSampleStrategy::Spatiotemporal;
      LOG(INFO) << "Event Sample Type: (3) spatiotemporal";
      break;
    default:
      EVENTAIL_WARN("Unknown Event Sample Type. Go with '(3) spatiotemporal' instead.");
      event_sample_type_ = EventSampleStrategy::Spatiotemporal;
      LOG(INFO) << "Event Sample Type: (3) spatiotemporal";
      break;
  }
  switch (FLAGS_NOISE_TYPE) {
    case 0:
      noise_type_ = NoiseModel::NoiseFree;
      LOG(INFO) << "Noise Type: (0) no noise";
      break;
    case 1:
      noise_type_ = NoiseModel::PixelNoise;
      LOG(INFO) << "Noise Type: (1) pixel noise";
      break;
    case 2:
      noise_type_ = NoiseModel::TimestampJitter;
      LOG(INFO) << "Noise Type: (2) timestamp jitter";
      break;
    case 3:
      noise_type_ = NoiseModel::AngularVelocityDisturbance;
      LOG(INFO) << "Noise Type: (3) angular velocity disturbance";
      break;
    default:
      EVENTAIL_WARN("Unknown Noise Type. Go with '(0) no noise' instead.");
      noise_type_ = NoiseModel::NoiseFree;
      LOG(INFO) << "Noise Type: (0) no noise";
      break;
  }
  switch (FLAGS_NOISE_DISTRIBUTION) {
    case 0:
      distribution_type_ = NoiseDistribution::Uniform;
      LOG(INFO) << "Noise Distribution: (0) Uniform Distribution";
      break;
    case 1:
      distribution_type_ = NoiseDistribution::Gaussian;
      LOG(INFO) << "Noise Distribution: (1) Gaussian Distribution";
      break;
    default:
      EVENTAIL_WARN("Unknown Noise Distribution. Go with '(0) Uniform Distribution' instead.");
      distribution_type_ = NoiseDistribution::Uniform;
      LOG(INFO) << "Noise Distribution: (0) Uniform Distribution";
      break;
  }
  if (noise_type_ != NoiseModel::NoiseFree) {
    for (double noise = FLAGS_MIN_NOISE_MAGNITUDE; noise <= FLAGS_MAX_NOISE_MAGNITUDE; noise += FLAGS_NOISE_STEP_SIZE)
      noise_levels_.emplace_back(noise);
    glogVector("Noise Level", noise_levels_);
  }

  // Averagor Setup
  switch (FLAGS_SVD_TYPE) {
    case 0:
      svd_type_ = SvdFunction::SVD;
      LOG(INFO) << "SVD Type: (0) SVD";
      break;
    case 1:
      svd_type_ = SvdFunction::PCA;
      LOG(INFO) << "SVD Type: (1) SVD with standardization";
      break;
    default:
      EVENTAIL_WARN("Unknown SVD Type. Go with '(1) SVD with standardization' instead.");
      svd_type_ = SvdFunction::PCA;
      LOG(INFO) << "SVD Type: (1) SVD with standardization";
      break;
  }
  linear_averagor_ = LinearAveragor(FLAGS_UNCERTAINTY_THRESHOLD, error_metric_type_, error_model_type_, svd_type_);
  polyjam_averagor_ = PolyjamAveragor(FLAGS_UNCERTAINTY_THRESHOLD, error_metric_type_, error_model_type_, svd_type_);
}

bool Simulator::isPointOnImage(Vector3d homo_pt) const {
  homo_pt /= homo_pt[2];
  return !(homo_pt[0] < 0.0 || homo_pt[0] > img_width_ - 1.0 || homo_pt[1] < 0.0 || homo_pt[1] > img_height_ - 1.0);
}

bool Simulator::isLineInFov(const Line3D& l, const Velocity& linear, const Velocity& angular) {
  Vector3d pt1_homo_begin =
      K_ * axisAngleToRotationMatrix(angular, time_window_.first).transpose() * (l.first - linear * time_window_.first);
  Vector3d pt2_homo_begin = K_ * axisAngleToRotationMatrix(angular, time_window_.first).transpose() *
                            (l.second - linear * time_window_.first);
  Vector3d pt1_homo_end = K_ * axisAngleToRotationMatrix(angular, time_window_.second).transpose() *
                          (l.first - linear * time_window_.second);
  Vector3d pt2_homo_end = K_ * axisAngleToRotationMatrix(angular, time_window_.second).transpose() *
                          (l.second - linear * time_window_.second);
  return (isPointOnImage(pt1_homo_begin) && isPointOnImage(pt2_homo_begin) && isPointOnImage(pt1_homo_end) &&
          isPointOnImage(pt2_homo_end));
}

// * At time zero, two feature are randomly selected and assigned random depth to create the two endpoints of a 3D line.
// * The line's validity within the time window is subsequently verified by checking whether it remains within the
// * camera's field of view.
Line3D Simulator::sampleLineBySceneToCamera(const Velocity& linear, const Velocity& angular) {
  Line3D l;
  do {
    // make sure two points are sampled on the left and right half of the image canvas
    Feature pt1 = generator_.sample2dPoint(int(img_width_) / 2, img_height_);
    Feature pt2 = generator_.sample2dPoint(int(img_width_) / 2, img_height_);
    pt2.x() += img_width_ / 2;
    // project sampled features to 3D with sampled depth
    Bearing bv1 = K_inv_ * pt1.homogeneous();
    Bearing bv2 = K_inv_ * pt2.homogeneous();
    bv1.normalize();
    bv2.normalize();
    Landmark lm1 = bv1 * generator_.sampleDepth(depth_range_);
    Landmark lm2 = bv2 * generator_.sampleDepth(depth_range_);
    // normalize to the X = +/-1 planes
    double ratio1 = (-1.0 - lm1[0]) / (lm2[0] - lm1[0]);
    double norm_ya = lm1[1] + ratio1 * (lm2[1] - lm1[1]);
    double norm_za = lm1[2] + ratio1 * (lm2[2] - lm1[2]);
    double ratio2 = (1.0 - lm1[0]) / (lm2[0] - lm1[0]);
    double norm_yb = lm1[1] + ratio2 * (lm2[1] - lm1[1]);
    double norm_zb = lm1[2] + ratio2 * (lm2[2] - lm1[2]);
    l = make_pair(Landmark{-1.0, norm_ya, norm_za}, Landmark{1.0, norm_yb, norm_zb});
  } while (isLineInFov(l, linear, angular));
  return l;
}

// * Two features are selected randomly at the beginning and end of the time window, representing the projected points
// * from the endpoints of the sampled 3D line. To ensure accuracy, it is subsequently verified that this 3D line is
// * solvable and not parallel to the yz plane.
Line3D Simulator::sampleLineByCameraToScene(const Velocity& linear, const Velocity& angular) {
  Line3D l;
  Position t_begin = linear * time_window_.first;
  Position t_end = linear * time_window_.second;
  RotationMatrix R_begin = axisAngleToRotationMatrix(angular, time_window_.first);
  RotationMatrix R_end = axisAngleToRotationMatrix(angular, time_window_.second);
  Normal n_begin, n_end;
  do {
    // compute the bearing vector of the features
    Feature pt1_begin = generator_.sample2dPoint(img_width_, img_height_);
    Feature pt2_begin = generator_.sample2dPoint(img_width_, img_height_);
    Feature pt1_end = generator_.sample2dPoint(img_width_, img_height_);
    Feature pt2_end = generator_.sample2dPoint(img_width_, img_height_);
    Bearing bv1_begin = R_begin * K_inv_ * pt1_begin.homogeneous();
    Bearing bv2_begin = R_begin * K_inv_ * pt2_begin.homogeneous();
    Bearing bv1_end = R_end * K_inv_ * pt1_end.homogeneous();
    Bearing bv2_end = R_end * K_inv_ * pt2_end.homogeneous();
    // compute the normal of each plane (composed of two bearing vector and one camera position)
    n_begin = bv1_begin.cross(bv2_begin);
    n_end = bv1_end.cross(bv2_end);
    n_begin.normalize();
    n_end.normalize();
  } while (!solveIntersectionOfPlanes(t_begin, n_begin, t_end, n_end, l));
  return l;
}

// * N events are sampled by first determining their timestamp and landmark on the 3D line, and then projecting them
// * onto the image plane at that particular time. Timestamp and landmark are both randomly sampled.
EventSet Simulator::sampleEventsByRandom(int N, const Velocity& linear, const Velocity& angular, const Line3D& line) {
  EventSet events;
  for (int idx = 0; idx < N; ++idx) {
    double ts = generator_.sampleTimestamp(time_window_);
    Landmark lm = line.first + generator_.sampleUniformScalar(0.0, 1.0) * (line.second - line.first);
    Vector3d event_homo = K_ * axisAngleToRotationMatrix(angular, ts).transpose() * (lm - linear * ts);
    event_homo /= event_homo[2];
    events.emplace_back(event_homo.x(), event_homo.y(), ts, 0);
  }
  generator_.shuffleEventSet(events);
  return events;
}

// * N events are sampled by first determining their timestamp and landmark on the 3D line, and then projecting them
// * onto the image plane at that particular time. The 3D line is projected onto the image plane, and each event is
// * randomly sampled within corresponding sub-segment. Timestamp is randomly sampled.
EventSet Simulator::sampleEventsBySpatial(int N, const Velocity& linear, const Velocity& angular, const Line3D& line) {
  EventSet events;
  for (int idx = 0; idx < N; ++idx) {
    double ts = generator_.sampleTimestamp(time_window_);
    Vector3d pt1_homo = K_ * axisAngleToRotationMatrix(angular, ts).transpose() * (line.first - linear * ts);
    Vector3d pt2_homo = K_ * axisAngleToRotationMatrix(angular, ts).transpose() * (line.second - linear * ts);
    pt1_homo /= pt1_homo[2];
    pt2_homo /= pt2_homo[2];
    Vector3d line_direction = pt2_homo - pt1_homo;
    double ratio1 = (0.0 - pt1_homo.x()) / line_direction.x();
    Vector3d normalized_pt1 = pt1_homo + ratio1 * line_direction;
    double ratio2 = ((img_width_ - 1.0) - pt2_homo.x()) / line_direction.x();
    Vector3d normalized_pt2 = pt2_homo + ratio2 * line_direction;
    Vector3d event_homo = normalized_pt1 + generator_.sampleUniformScalar((1.0 / N) * idx, (1.0 / N) * (idx + 1)) *
                                               (normalized_pt2 - normalized_pt1);
    events.emplace_back(event_homo.x(), event_homo.y(), ts, 0);
  }
  generator_.shuffleEventSet(events);
  return events;
}

// * N events are sampled by first determining their timestamp and landmark on the 3D line, and then projecting them
// * onto the image plane at that particular time. Time interval is evenly divided into five sub-intervals, and
// * timestamp is randomly sampled within corresponding sub-interval. Landmark is randomly sampled.
EventSet Simulator::sampleEventsByTemporal(int N, const Velocity& linear, const Velocity& angular, const Line3D& line) {
  EventSet events;
  double slice = (time_window_.second - time_window_.first) / N;
  for (int idx = 0; idx < N; ++idx) {
    double_pair curr_time_window = make_pair(time_window_.first + slice * idx, time_window_.first + slice * (idx + 1));
    double ts = generator_.sampleTimestamp(curr_time_window);
    Landmark lm = line.first + generator_.sampleUniformScalar(0.0, 1.0) * (line.second - line.first);
    Vector3d event_homo = K_ * axisAngleToRotationMatrix(angular, ts).transpose() * (lm - linear * ts);
    event_homo /= event_homo[2];
    events.emplace_back(event_homo.x(), event_homo.y(), ts, 0);
  }
  generator_.shuffleEventSet(events);
  return events;
}

// * N events are sampled by first determining their timestamp and landmark on the 3D line, and then projecting them
// * onto the image plane at that particular time. Time interval is evenly divided into five sub-intervals, and
// * timestamp is randomly sampled within corresponding sub-interval. The 3D line is projected onto the image plane, and
// * each event is randomly sampled within corresponding sub-segment.
EventSet Simulator::sampleEventsBySpatiotemporal(int N, const Velocity& linear, const Velocity& angular,
                                                 const Line3D& line) {
  EventSet events;
  double slice = (time_window_.second - time_window_.first) / N;
  vector<int> shuffle_time_idx = generator_.sampleShuffledRange(N);
  for (int idx = 0; idx < N; ++idx) {
    double_pair curr_time_window = make_pair(time_window_.first + slice * shuffle_time_idx[idx],
                                             time_window_.first + slice * (shuffle_time_idx[idx] + 1));
    double ts = generator_.sampleTimestamp(curr_time_window);
    Vector3d pt1_homo = K_ * axisAngleToRotationMatrix(angular, ts).transpose() * (line.first - linear * ts);
    Vector3d pt2_homo = K_ * axisAngleToRotationMatrix(angular, ts).transpose() * (line.second - linear * ts);
    pt1_homo /= pt1_homo[2];
    pt2_homo /= pt2_homo[2];
    Vector3d line_direction = pt2_homo - pt1_homo;
    double ratio1 = (0.0 - pt1_homo.x()) / line_direction.x();
    Vector3d normalized_pt1 = pt1_homo + ratio1 * line_direction;
    double ratio2 = ((img_width_ - 1.0) - pt2_homo.x()) / line_direction.x();
    Vector3d normalized_pt2 = pt2_homo + ratio2 * line_direction;
    Vector3d event_homo = normalized_pt1 + generator_.sampleUniformScalar((1.0 / N) * idx, (1.0 / N) * (idx + 1)) *
                                               (normalized_pt2 - normalized_pt1);
    events.emplace_back(event_homo.x(), event_homo.y(), ts, 0);
  }
  generator_.shuffleEventSet(events);
  return events;
}

EventSet Simulator::addPixelNoise(const EventSet& events, double noise_level) {
  EventSet noisy_events;
  for (const auto& e : events) {
    Vector2d pixel_noise;
    if (distribution_type_ == NoiseDistribution::Gaussian)
      pixel_noise = generator_.sampleGaussianPixelNoise(noise_level);
    else  // ! default choice (NoiseDistribution::Uniform)
      pixel_noise = generator_.sampleUniformPixelNoise(noise_level);
    noisy_events.emplace_back(e.x + pixel_noise.x(), e.y + pixel_noise.y(), e.t, e.p);
  }
  return noisy_events;
}

EventSet Simulator::addTimestampJitter(const EventSet& events, double noise_level) {
  EventSet noisy_events;
  for (const auto& e : events) {
    if (distribution_type_ == NoiseDistribution::Uniform)
      noisy_events.emplace_back(e.x, e.y, e.t + generator_.sampleUniformTimestampJitter(noise_level), e.p);
    else  // ! default choice (NoiseDistribution::Gaussian)
      noisy_events.emplace_back(e.x, e.y, e.t + generator_.sampleGaussianTimestampJitter(noise_level), e.p);
  }

  return noisy_events;
}

Velocity Simulator::addAngularVelocityDisturbance(const Velocity& angular, double noise_level) {
  if (distribution_type_ == NoiseDistribution::Gaussian)
    return (angular + generator_.sampleGaussianAngularVelocityDisturbance(deg2rad(noise_level)));
  else  // ! default choice (NoiseDistribution::Uniform)
    return (angular + generator_.sampleUniformAngularVelocityDisturbance(deg2rad(noise_level)));
}

Velocity Simulator::sampleLinearVelocity() { return generator_.sampleLinearVelocity(lin_vel_range_); }

Velocity Simulator::sampleAngularVelocity() { return generator_.sampleAngularVelocity(ang_vel_range_); }

Line3D Simulator::sampleLine(const Velocity& linear, const Velocity& angular) {
  if (line_type_ == LineModel::Scene2Camera)
    return sampleLineBySceneToCamera(linear, angular);
  else  // ! default choice (LineModel::Camera2Scene)
    return sampleLineByCameraToScene(linear, angular);
}

EventSet Simulator::sampleRandomEvents(int N) {
  EventSet events;
  for (int idx = 0; idx < N; ++idx)
    events.emplace_back(generator_.sampleRandomEvent(img_width_, img_height_, time_window_));
  return events;
}

EventSet Simulator::sampleEvents(int N, const Velocity& linear, const Velocity& angular, const Line3D& line) {
  if (event_sample_type_ == EventSampleStrategy::Random)
    return sampleEventsByRandom(N, linear, angular, line);
  else if (event_sample_type_ == EventSampleStrategy::Spatial)
    return sampleEventsBySpatial(N, linear, angular, line);
  else if (event_sample_type_ == EventSampleStrategy::Temporal)
    return sampleEventsByTemporal(N, linear, angular, line);
  else  // ! default choice (EventSampleStrategy::Spatiotemporal)
    return sampleEventsBySpatiotemporal(N, linear, angular, line);
}

EventSet Simulator::sampleFiveEvents(const Velocity& linear, const Velocity& angular, const Line3D& line) {
  return sampleEvents(5, linear, angular, line);
}

EventSet Simulator::sampleTenEvents(const Velocity& linear, const Velocity& angular, const Line3D& line) {
  return sampleEvents(10, linear, angular, line);
}

Measurements Simulator::sampleNoisyMeasurements(const EventSet& events, const Velocity& angular, double noise_level) {
  if (noise_type_ == NoiseModel::PixelNoise)
    return make_pair(addPixelNoise(events, noise_level), angular);
  else if (noise_type_ == NoiseModel::TimestampJitter)
    return make_pair(addTimestampJitter(events, noise_level), angular);
  else if (noise_type_ == NoiseModel::AngularVelocityDisturbance)
    return make_pair(events, addAngularVelocityDisturbance(angular, noise_level));
  else  // ! default choice (NoiseModel::NoiseFree)
    return make_pair(events, angular);
}

void Simulator::setupGroundTruth(const Velocity& linear, const Line3D& line) {
  linear_solver_.inputGroundTruth(line.first, line.second, linear);
  polyjam_solver_.inputGroundTruth(line.first, line.second, linear);
  linear_averagor_.inputGroundTruth(linear);
  polyjam_averagor_.inputGroundTruth(linear);
}

bool Simulator::linearSolve(const Measurements& measurements, double& duration, double& error, bool uncertainty,
                            bool standardization) {
  if (!linear_solver_.runSolver(measurements, duration, standardization)) return false;
  if (uncertainty)
    if (!linear_solver_.calculateCovariance(measurements)) return false;
  linear_averagor_.addData(linear_solver_.getSolution());
  error = linear_solver_.getError();
  return true;
}

bool Simulator::polyjamSolve(const Measurements& measurements, double& duration, double& error, bool uncertainty,
                             bool standardization) {
  if (!polyjam_solver_.runSolver(measurements, duration, standardization)) return false;
  if (uncertainty)
    if (!polyjam_solver_.calculateCovariance(measurements)) return false;
  polyjam_averagor_.addData(polyjam_solver_.getSolution());
  error = polyjam_solver_.getError();
  return true;
}

bool Simulator::linearAverage(Velocity& linear_estimation, double& error, bool reset) {
  if (!linear_averagor_.runAveragor()) return false;
  linear_estimation = linear_averagor_.getEstimation();
  error = linear_averagor_.getError();
  if (reset) linear_averagor_.reset();
  return true;
}

bool Simulator::polyjamAverage(Velocity& linear_estimation, double& error, bool reset) {
  if (!polyjam_averagor_.runAveragor()) return false;
  linear_estimation = polyjam_averagor_.getEstimation();
  error = polyjam_averagor_.getError();
  if (reset) polyjam_averagor_.reset();
  return true;
}

}  // namespace eventail_simulation
