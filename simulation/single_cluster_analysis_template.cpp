#include "eventail_simulation/analyzer.hpp"
#include "eventail_simulation/simulator.hpp"

using namespace eventail_simulation;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_log_dir = (std::filesystem::path{__FILE__}.parent_path().parent_path() / "log").string();

  // initialize the simulator
  auto sim = Simulator();

  // loop over various noise level
  auto noise_level_vec = sim.getNoiseLevels();
  for (auto noise_level : noise_level_vec) {
    auto analyzer = Analyzer(sim.getSceneNum());

    // motion-geometry configuration
    for (int scene_idx = 0; scene_idx < sim.getSceneNum(); ++scene_idx) {
      Velocity lin_vel = sim.sampleLinearVelocity();
      Velocity ang_vel = sim.sampleAngularVelocity();
      Line3D l = sim.sampleLine(lin_vel, ang_vel);
      sim.setupGroundTruth(lin_vel, l);

      // sample random, noisy measurements
      double error_sum = 0.0;
      for (int noise_idx = 0; noise_idx < sim.getNoiseNum(); ++noise_idx) {
        EventSet clean_events = sim.sampleFiveEvents(lin_vel, ang_vel, l);
        Measurements noisy_measurements = sim.sampleNoisyMeasurements(clean_events, ang_vel, noise_level);

        // execute the linear solver and record errors
        double duration = 0.0, error = 0.0;
        if (!sim.linearSolve(noisy_measurements, duration, error)) {
          EVENTAIL_WARN("Encounter unsolvable degenerate case!");
          --noise_idx;
        }
        error_sum += error;
      }
      analyzer.addData(error_sum / sim.getNoiseNum());
    }

    // analyze the performance
    analyzer.analysis("Performance Analysis on Linear Solver, with the noise level of " + std::to_string(noise_level));
  }

  return 0;
}
