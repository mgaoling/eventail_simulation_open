#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#define EVENTAIL_NEWLINE() (std::cout << std::endl)
#define EVENTAIL_PLAIN(x) (std::cout << ">> " + std::string(x) << std::endl)
#define EVENTAIL_INFO(x) (std::cout << "\033[1;32m>> " + std::string(x) + " \033[0m" << std::endl)
#define EVENTAIL_WARN(x) (std::cout << "\033[1;35m>> " + std::string(x) + " \033[0m" << std::endl)
#define EVENTAIL_ERROR(x) (std::cerr << "\033[1;31m>> [ERROR] " + std::string(x) + " \033[0m" << std::endl)

namespace eventail_simulation {

enum class ErrorMetric { DirectionError, EuclideanDistance };
enum class ErrorModel { MotionOnly, MotionAndGeometry };
enum class LineModel { Scene2Camera, Camera2Scene };
enum class EventSampleStrategy { Random, Spatial, Temporal, Spatiotemporal };
enum class NoiseModel { NoiseFree, PixelNoise, TimestampJitter, AngularVelocityDisturbance };
enum class NoiseDistribution { Uniform, Gaussian };
enum class SvdFunction { SVD, PCA };

struct Event {
  using Ptr = std::shared_ptr<Event>;

  uint32_t x{};
  uint32_t y{};
  double t{};
  bool p{};

  Event() = default;
  Event(uint32_t px, uint32_t py, double ts, bool polarity) : x(px), y(py), t(ts), p(polarity){};
};

struct Eventd {
  using Ptr = std::shared_ptr<Eventd>;

  double x;
  double y;
  double t;
  bool p;

  Eventd() = default;
  Eventd(double px, double py, double ts, bool polarity) : x(px), y(py), t(ts), p(polarity){};
};

// Arbitrarily Small Value
const double EPSILON0 = 1e-0;
const double EPSILON1 = 1e-1;
const double EPSILON3 = 1e-3;
const double EPSILON6 = 1e-6;
const double EPSILON9 = 1e-9;

// C++ Standard
using std::abs;
using std::begin;
using std::cos;
using std::cout;
using std::end;
using std::endl;
using std::isnan;
using std::make_pair;
using std::max;
using std::min;
using std::pow;
using std::shared_ptr;
using std::sin;
using std::sqrt;
using std::string;
using std::vector;

// Eigen
using AngleAxisd = Eigen::AngleAxisd;
using Quaterniond = Eigen::Quaterniond;
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using VectorXd = Eigen::VectorXd;
using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using MatrixXd = Eigen::MatrixXd;

using Time = double;
using Feature = Eigen::Vector2d;
using Basis = Eigen::Vector3d;
using Bearing = Eigen::Vector3d;
using Normal = Eigen::Vector3d;
using Position = Eigen::Vector3d;
using Landmark = Eigen::Vector3d;
using Velocity = Eigen::Vector3d;
using RotationMatrix = Eigen::Matrix3d;
using LinearSolution = Eigen::Matrix<double, 3, 5>;
using PolyjamSolution = Eigen::Matrix<double, 3, 4>;
using FiveEventMatrix = Eigen::Matrix<double, 2, 5>;
using CovarianceMatrix = Eigen::Matrix<double, 5, 5>;

// Complex Object
using double_pair = std::pair<double, double>;
using Line2D = std::pair<Feature, Feature>;
using Line3D = std::pair<Landmark, Landmark>;
using Line2DPair = std::pair<Line2D, Line2D>;
using EventSet = std::vector<Eventd>;
using Measurements = std::pair<EventSet, Velocity>;
using FullLinearSolution = std::pair<LinearSolution, CovarianceMatrix>;
using FullPolyjamSolution = std::pair<PolyjamSolution, CovarianceMatrix>;

}  // namespace eventail_simulation
