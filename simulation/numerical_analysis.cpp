#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim = Simulator();
  size_t num_exp = sim.getSceneNum() * sim.getNoiseNum();
  std::cout << num_exp << std::endl;

  // numerical unstable case counters
  size_t linear_error_counter_01 = 0, linear_error_counter_1 = 0;
  size_t polyjam_error_counter_01 = 0, polyjam_error_counter_1 = 0;

  // motion-geometry configuration
  for (int scene_idx = 0; scene_idx < sim.getSceneNum(); ++scene_idx) {
    Velocity lin_vel = sim.sampleLinearVelocity();
    Velocity ang_vel = sim.sampleAngularVelocity();
    Line3D l = sim.sampleLine(lin_vel, ang_vel);
    sim.setupGroundTruth(lin_vel, l);

    // we keep the usual noise_num & noisy_measurement structure here, but without adding any noise
    for (int noise_idx = 0; noise_idx < sim.getNoiseNum(); ++noise_idx) {
      EventSet clean_events = sim.sampleFiveEvents(lin_vel, ang_vel, l);
      Measurements measurements = sim.sampleNoisyMeasurements(clean_events, ang_vel, 0.0);

      // execute both polyjam and linear solver
      double linear_duration, polyjam_duration, linear_error, polyjam_error;
      if (!sim.linearSolve(measurements, linear_duration, linear_error)) {
        EVENTAIL_WARN("Encounter unsolvable degenerate case!");
        --noise_idx;
        continue;
      }
      if (!sim.polyjamSolve(measurements, polyjam_duration, polyjam_error)) {
        EVENTAIL_WARN("Encounter unsolvable degenerate case!");
        --noise_idx;
        continue;
      }

      // record the numerical unstable case
      if (linear_error > EPSILON0) ++linear_error_counter_1;
      if (linear_error > EPSILON1) ++linear_error_counter_01;
      if (polyjam_error > EPSILON0) ++polyjam_error_counter_1;
      if (polyjam_error > EPSILON1) ++polyjam_error_counter_01;
    }
  }

  // analyze the numerical instability
  EVENTAIL_INFO("Numerical Analysis (> 1.0 deg)");
  std::cout << "Linear Solver: " << 100.0 * linear_error_counter_1 / num_exp << "%" << std::endl;
  std::cout << "Polyjam Solver: " << 100.0 * polyjam_error_counter_1 / num_exp << "%" << std::endl;
  LOG(INFO) << "Numerical Analysis (> 1.0 deg)";
  LOG(INFO) << "Linear Solver: " << 100.0 * linear_error_counter_1 / num_exp << "%";
  LOG(INFO) << "Polyjam Solver: " << 100.0 * polyjam_error_counter_1 / num_exp << "%";

  EVENTAIL_INFO("Numerical Analysis (> 0.1 deg)");
  std::cout << "Linear Solver: " << 100.0 * linear_error_counter_01 / num_exp << "%" << std::endl;
  std::cout << "Polyjam Solver: " << 100.0 * polyjam_error_counter_01 / num_exp << "%" << std::endl;
  LOG(INFO) << "Numerical Analysis (> 0.1 deg)";
  LOG(INFO) << "Linear Solver: " << 100.0 * linear_error_counter_01 / num_exp << "%";
  LOG(INFO) << "Polyjam Solver: " << 100.0 * polyjam_error_counter_01 / num_exp << "%";

  return 0;
}
